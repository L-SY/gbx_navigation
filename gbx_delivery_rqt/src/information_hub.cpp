//
// Created by lsy on 24-12-3.
//

#include "gbx_delivery_rqt/information_hub.h"
#include <QDir>
#include <QCoreApplication>
#include <sstream>

namespace gbx_rqt_interact {

InformationHub::InformationHub(QObject *parent)
    : QThread(parent),
      serial(nullptr),
      updateTimer(nullptr),
      nh_(nullptr)
{
  memset(door_state, 0, sizeof(door_state));
  memset(door_set_state, 0, sizeof(door_set_state));
  memset(last_door_state, 0, sizeof(last_door_state));
}

InformationHub::~InformationHub()
{
  requestInterruption();
  wait();

  if (serial && serial->isOpen()) {
    serial->close();
  }
  delete serial;
  delete nh_;
  if (updateTimer) {
    updateTimer->stop();
    delete updateTimer;
  }
}

// ----------------------------Init Related Start----------------------------
void InformationHub::init()
{
  // Initialize serial port
  serial = new QSerialPort(this);

  // Initialize timer for serial data processing
  updateTimer = new QTimer(this);
  updateTimer->setInterval(20); // 50Hz update frequency
  connect(updateTimer, &QTimer::timeout, this, &InformationHub::processSerialData);

  initializeDevice();

  try {
    initializeROS();
    // 加载目标点数据
    QString resourcePath = QCoreApplication::applicationDirPath() + "/resources/2F_whole_finalpoint.csv";
    if (!loadTargetPoints(resourcePath.toStdString())) {
      ROS_WARN_STREAM("Failed to load target points from: " << resourcePath.toStdString());
      // 尝试相对路径
      if (!loadTargetPoints("/home/vipbot/ros_ws/navigation_ws/src/gbx_navigation/gbx_delivery_rqt/resources/2F_whole_finalpoint.csv")) {
        ROS_ERROR_STREAM("Failed to load target points from relative path");
      }
    }
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("ROS initialization failed, but continuing with other functions: " << e.what());
  }

  updateTimer->start();
}

void InformationHub::initializeROS()
{
  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = nullptr;
    ros::init(argc, argv, "gbx_rqt_interact_node", ros::init_options::NoSigintHandler);
  }
  if (!nh_) {
    nh_ = new ros::NodeHandle("~");
  }

  trajectory_client_ = nh_->serviceClient<navigation_msgs::pub_trajectory>("/gbx_manual/pub_trajectory");
  door_state_pub_ = nh_->advertise<navigation_msgs::CabinetDoorArray>("/cabinet/door_states", 1);
  indoor_delivery_pub_ = nh_->advertise<navigation_msgs::IndoorDeliveryOrder>("/indoor_delivery_order", 1);
  output_delivery_pub_ = nh_->advertise<navigation_msgs::OutputDelivery>("/output_delivery", 1);
  goal_pub_ = nh_->advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
  cabinet_content_sub_ = nh_->subscribe("/cabinet/contents", 1,&InformationHub::cabinetContentCallback, this);
  navigation_arrived_sub_ = nh_->subscribe("/move_base/result", 1, &InformationHub::navigationArrivedCallback, this);
}

// ----------------------------Init Related End----------------------------

void InformationHub::run()
{
  while (!isInterruptionRequested()) {
    if (nh_) {
      ros::spinOnce();
    }
    msleep(20); // Reduce thread frequency to 50Hz
  }
}

// ----------------------------Serial(Door) Related Start----------------------------
void InformationHub::initializeDevice()
{
  if (!serial) {
    serial = new QSerialPort(this);
  }
}

bool InformationHub::openSerialPort(const QString& portName)
{
  if (!serial) {
    serial = new QSerialPort(this);
  }

  serial->setPortName(portName);
  serial->setBaudRate(QSerialPort::Baud115200);
  serial->setDataBits(QSerialPort::Data8);
  serial->setParity(QSerialPort::NoParity);
  serial->setStopBits(QSerialPort::OneStop);
  serial->setFlowControl(QSerialPort::NoFlowControl);

  if (serial->open(QIODevice::ReadWrite)) {
    serial->setDataTerminalReady(true);
    ROS_INFO_STREAM("Serial port opened successfully");
    return true;
  } else {
    ROS_ERROR_STREAM("Failed to open serial port!");
    return false;
  }
}

void InformationHub::closeSerialPort()
{
  if (serial && serial->isOpen()) {
    serial->close();
  }
}

bool InformationHub::isSerialPortOpen() const
{
  return serial && serial->isOpen();
}

void InformationHub::processSerialData()
{
  readSerialData();
}

void InformationHub::readSerialData()
{
  if (serial && serial->isOpen() && serial->bytesAvailable() >= 8) {
    uint8_t new_state[8];
    serial->read((char*)new_state, 8);

    // Check if data has changed
    if (memcmp(new_state, door_state, 8) != 0) {
      memcpy(door_state, new_state, 8);
      emit doorStateChanged();
      publishDoorStates();
    }
  }
}

void InformationHub::sendDoorCommand(int boxIndex)
{
  if (serial && serial->isOpen()) {
    memset(door_set_state, 0, sizeof(door_set_state));
    door_set_state[boxIndex] = 1;
    serial->write((char*)door_set_state, 6);
    ROS_INFO_STREAM("Opening Box " << (boxIndex + 1));
  }
}

void InformationHub::publishDoorStates()
{
  navigation_msgs::CabinetDoorArray msg;
  msg.header.stamp = ros::Time::now();

  for (int i = 0; i < 6; ++i) {
    navigation_msgs::CabinetDoor door;
    door.id = "door_" + std::to_string(i + 1);
    door.is_open = (door_state[i + 1] == 1);
    door.last_changed = ros::Time::now();
    msg.doors.push_back(door);
  }

  door_state_pub_.publish(msg);
}
// ----------------------------Serial(Door) Related End----------------------------


bool InformationHub::hasStateChanged() const
{
  return memcmp(door_state, last_door_state, 8) != 0;
}

bool InformationHub::hasContentsChanged() const
{
  if (current_contents_.size() != last_contents_.size()) {
    return true;
  }

  for (size_t i = 0; i < current_contents_.size(); ++i) {
    if (current_contents_[i].box.ascii_epc != last_contents_[i].box.ascii_epc) {
      return true;
    }
  }
  return false;
}



void InformationHub::cabinetContentCallback(const navigation_msgs::CabinetContentArray::ConstPtr& msg)
{
  current_contents_ = msg->cabinets;
  emit contentStateChanged();
}


// ----------------------------Navigation Related Start----------------------------
bool InformationHub::sendTrajectoryRequest(const QString& path_name)
{
  if (!nh_ || !trajectory_client_) {
    emit trajectoryRequestResult(false, "ROS service not available");
    return false;
  }

  navigation_msgs::pub_trajectory srv;
  srv.request.sender = "rqt_interact";
  srv.request.path_name = path_name.toStdString();

  if (trajectory_client_.call(srv)) {
    emit trajectoryRequestResult(srv.response.success,
                                 QString::fromStdString(srv.response.message));
    return true;
  } else {
    emit trajectoryRequestResult(false, "Failed to call trajectory service");
    return false;
  }
}

bool InformationHub::loadTargetPoints(const std::string& csvPath)
{
  std::ifstream file(csvPath);
  if (!file.is_open()) {
    ROS_ERROR_STREAM("Failed to open CSV file: " << csvPath);
    return false;
  }

  target_points_.clear();
  std::string line;
  // 跳过标题行
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ',')) {
      tokens.push_back(token);
    }

    if (tokens.size() >= 4) {
      TargetPoint point;
      try {
        point.index = std::stoi(tokens[0]);
        point.x = std::stod(tokens[1]);
        point.y = std::stod(tokens[2]);
        point.z = std::stod(tokens[3]);
        target_points_.push_back(point);
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error parsing point data: " << e.what());
        continue;
      }
    }
  }

  ROS_INFO_STREAM("Loaded " << target_points_.size() << " target points");
  return !target_points_.empty();
}

bool InformationHub::sendTargetPoint(int pointIndex)
{
  if (!nh_ || !goal_pub_) {
    emit trajectoryRequestResult(false, "ROS publisher not available");
    return false;
  }

  // 检查点索引是否有效
  if (pointIndex < 0 || pointIndex >= static_cast<int>(target_points_.size())) {
    emit trajectoryRequestResult(false, "Invalid point index");
    return false;
  }

  const TargetPoint& point = target_points_[pointIndex];

  geometry_msgs::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.header.stamp = ros::Time::now();
  goal.pose.position.x = point.x;
  goal.pose.position.y = point.y;
  goal.pose.position.z = point.z;
  goal.pose.orientation.w = 1.0;  // 默认朝向

  goal_pub_.publish(goal);
  emit trajectoryRequestResult(true, "Goal published successfully");
  return true;
}
// ----------------------------Navigation Related End----------------------------

void InformationHub::publishIndoorDeliveryOrder(
    const std::string& carNumber,
    const std::string& rfid,
    const std::string& converted_rfid,
    const std::string& owner,
    const std::string& area,
    const std::string& orderNumber,
    const std::string& receiverPhone,
    const std::string& receiverName,
    const std::string& senderName)
{
  navigation_msgs::IndoorDeliveryOrder msg;

  // 设置所有字段
  msg.Number = carNumber;            // 车号
  msg.Owner = owner;                 // 箱子所有者
  msg.Area = area;                   // 区域位置
  msg.RFID = rfid;                  // 原始RFID
  msg.Converted_RFID = converted_rfid;        // 转换后的RFID（这里保持原值）
  msg.ReceiverPhone = receiverPhone; // 接收者电话
  msg.OrderNumber = orderNumber;     // 订单号（默认为0）
  msg.ReceiverName = receiverName;  // 接收者姓名
  msg.SenderName = senderName;      // 发送者姓名

  // 发布消息
  indoor_delivery_pub_.publish(msg);
  ROS_INFO_STREAM("Published IndoorDeliveryOrder: "
                  << "CarNumber=" << carNumber
                  << " RFID=" << rfid
                  << " Area=" << area);
}

void InformationHub::publishOutputDelivery(
    const std::string& owner,
    const std::string& rfid,
    const std::string& converted_rfid,
    const std::string& receiverPhone)
{
  navigation_msgs::OutputDelivery msg;

  msg.Owner = owner;
  msg.RFID = rfid;
  msg.Converted_RFID = converted_rfid;
  msg.ReceiverPhone = receiverPhone;

  output_delivery_pub_.publish(msg);
  ROS_INFO_STREAM("Published OutputDelivery: "
                  << "Owner=" << owner
                  << " RFID=" << rfid);
}

void InformationHub::navigationArrivedCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg) {
  if (msg->status.status == 3)
  {
    emit navigationArrived(true);
    ROS_INFO_STREAM("ARRIVAL!!!!!!!");
  }
}
} // namespace gbx_rqt_interact
