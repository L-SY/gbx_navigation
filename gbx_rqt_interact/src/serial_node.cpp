#include "gbx_rqt_interact/serial_node.h"
#include "ui_main_window.h"
#include <QTime>
#include <QDebug>

namespace gbx_rqt_interact {

SerialNode::SerialNode(QObject *parent)
    : QThread(parent),
      mainWindow_ui(nullptr),
      serial(nullptr),
      updateTimer(nullptr),
      show_box_flag(false),
      show_dest_flag(false),
      send_set_flag(false),
      button_update_flag(false),
      nh_(nullptr)
{
  memset(box_fdb_state, 0, sizeof(box_fdb_state));
  memset(box_set_state, 0, sizeof(box_set_state));
  memset(last_box_state, 0, sizeof(last_box_state));
}

SerialNode::~SerialNode()
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

void SerialNode::Init(Ui::MainWindow* _mainWindow_ui)
{
  mainWindow_ui = _mainWindow_ui;
  serial = new QSerialPort(this);

  // 创建定时器用于串口数据处理
  updateTimer = new QTimer(this);
  updateTimer->setInterval(20); // 50Hz更新频率
  connect(updateTimer, &QTimer::timeout, this, &SerialNode::processSerialData);

  DeviceInit();

  try {
    ROSInit();
  } catch (const std::exception& e) {
    qDebug() << "ROS initialization failed, but continuing with other functions";
  }

  UiInit();
  connect(this, &SerialNode::requestUIUpdate, this, &SerialNode::UpdateUI, Qt::QueuedConnection);

  updateTimer->start();
}

void SerialNode::DeviceInit()
{
  if (!serial) {
    serial = new QSerialPort(this);
  }

  serial->setPortName("/dev/ttyACM0");
  serial->setBaudRate(QSerialPort::Baud115200);
  serial->setDataBits(QSerialPort::Data8);
  serial->setParity(QSerialPort::NoParity);
  serial->setStopBits(QSerialPort::OneStop);
  serial->setFlowControl(QSerialPort::NoFlowControl);

  if (serial->open(QIODevice::ReadWrite)) {
    serial->setDataTerminalReady(true);
    qDebug() << "Serial port opened successfully";
  } else {
    qDebug() << "Failed to open serial port!";
  }
}

void SerialNode::ROSInit()
{
  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = nullptr;
    ros::init(argc, argv, "gbx_rqt_interact_node", ros::init_options::NoSigintHandler);
  }
  if (!nh_) {
    nh_ = new ros::NodeHandle("~");
  }

  trajectory_client_ = nh_->serviceClient<navigation_msgs::pub_trajectory>("/pub_trajectory");
  door_state_pub_ = nh_->advertise<navigation_msgs::CabinetDoorArray>("/cabinet/door_states", 1);
  cabinet_content_sub_ = nh_->subscribe("/cabinet/contents", 1,
                                        &SerialNode::cabinetContentCallback, this);
}

void SerialNode::UiInit()
{
  // 一次性连接所有按钮
  connect(mainWindow_ui->boxButton, &QPushButton::clicked,
          this, &SerialNode::handleBoxButtonClick, Qt::QueuedConnection);
  connect(mainWindow_ui->destButton, &QPushButton::clicked,
          this, &SerialNode::handleDestButtonClick, Qt::QueuedConnection);

  // 使用统一的处理函数处理按钮点击
  connect(mainWindow_ui->B1, &QPushButton::clicked,
          this, [this](){ handleButtonClick(0); }, Qt::QueuedConnection);
  connect(mainWindow_ui->B2, &QPushButton::clicked,
          this, [this](){ handleButtonClick(1); }, Qt::QueuedConnection);
  connect(mainWindow_ui->B3, &QPushButton::clicked,
          this, [this](){ handleButtonClick(2); }, Qt::QueuedConnection);
  connect(mainWindow_ui->B4, &QPushButton::clicked,
          this, [this](){ handleButtonClick(3); }, Qt::QueuedConnection);
  connect(mainWindow_ui->B5, &QPushButton::clicked,
          this, [this](){ handleButtonClick(4); }, Qt::QueuedConnection);
  connect(mainWindow_ui->B6, &QPushButton::clicked,
          this, [this](){ handleButtonClick(5); }, Qt::QueuedConnection);
}

void SerialNode::handleButtonClick(int buttonIndex)
{
  if (show_box_flag) {
    sendBoxCommand(buttonIndex);
  } else if (show_dest_flag) {
    sendTrajectoryRequest(labels[buttonIndex]);
  }
}

void SerialNode::handleBoxButtonClick()
{
  show_box_flag = true;
  show_dest_flag = false;
  emit requestUIUpdate();
}

void SerialNode::handleDestButtonClick()
{
  show_box_flag = false;
  show_dest_flag = true;
  emit requestUIUpdate();
}

void SerialNode::run()
{
  while (!isInterruptionRequested()) {
    if (nh_) {
      ros::spinOnce();
    }
    msleep(20); // 降低线程频率到50Hz
  }
}

void SerialNode::processSerialData()
{
  readSerialData();
}

void SerialNode::readSerialData()
{
  if (serial && serial->isOpen() && serial->bytesAvailable() >= 8) {
    uint8_t new_state[8];
    serial->read((char*)new_state, 8);

    // 检查数据是否发生变化
    if (memcmp(new_state, box_fdb_state, 8) != 0) {
      memcpy(box_fdb_state, new_state, 8);
      emit requestUIUpdate();
      publishDoorStates();
    }
  }
}

bool SerialNode::hasStateChanged() const
{
  return memcmp(box_fdb_state, last_box_state, 8) != 0;
}

bool SerialNode::hasContentsChanged() const
{
  if (current_contents_.size() != last_contents_.size()) {
    return true;
  }

  for (size_t i = 0; i < current_contents_.size(); ++i) {
    if (current_contents_[i].box.box_id != last_contents_[i].box.box_id) {
      return true;
    }
  }
  return false;
}

void SerialNode::updateButton(int index)
{
  QPushButton* buttons[6] = {mainWindow_ui->B1, mainWindow_ui->B2, mainWindow_ui->B3,
                             mainWindow_ui->B4, mainWindow_ui->B5, mainWindow_ui->B6};

  if (show_box_flag) {
    QString buttonText;
    std::string cabinet_id = "cabinet_" + std::to_string(index + 1);

    auto it = std::find_if(current_contents_.begin(), current_contents_.end(),
                           [&](const navigation_msgs::CabinetContent& content) {
                             return content.cabinet_id == cabinet_id;
                           });

    if (box_fdb_state[index+1] == 1) {
      buttons[index]->setStyleSheet("color: #A367CA;");
    } else {
      buttons[index]->setStyleSheet("color: #091648;");
    }

    if (it != current_contents_.end() && !it->box.box_id.empty()) {
      buttonText = QString("Box %1\n%2")
                       .arg(index + 1)
                       .arg(QString::fromStdString(it->box.box_id));
    } else {
      buttonText = QString("Box %1\n%2")
                       .arg(index + 1)
                       .arg(box_fdb_state[index+1] == 1 ? "OPEN" : "CLOSE");
    }
    buttons[index]->setText(buttonText);
  } else if (show_dest_flag) {
    buttons[index]->setStyleSheet("color: rgb(0,0,0)");
    buttons[index]->setText(labels[index]);
  }
}

void SerialNode::UpdateUI()
{
  bool stateChanged = hasStateChanged();
  bool contentsChanged = hasContentsChanged();

  if (stateChanged || contentsChanged || show_box_flag != button_update_flag) {
    for (int i = 0; i < 6; ++i) {
      updateButton(i);
    }

    // 更新状态记录
    memcpy(last_box_state, box_fdb_state, 8);
    last_contents_ = current_contents_;
    button_update_flag = show_box_flag;
  }
}

void SerialNode::publishDoorStates()
{
  navigation_msgs::CabinetDoorArray msg;
  msg.header.stamp = ros::Time::now();

  for (int i = 0; i < 6; ++i) {
    navigation_msgs::CabinetDoor door;
    door.id = "door_" + std::to_string(i + 1);
    door.is_open = (box_fdb_state[i + 1] == 1);
    door.status = door.is_open ? "open" : "closed";
    door.last_changed = ros::Time::now();
    msg.doors.push_back(door);
  }

  door_state_pub_.publish(msg);
}

void SerialNode::cabinetContentCallback(const navigation_msgs::CabinetContentArray::ConstPtr& msg)
{
  current_contents_ = msg->cabinets;
  emit requestUIUpdate();
}

void SerialNode::sendBoxCommand(int boxIndex)
{
  if (serial && serial->isOpen()) {
    memset(box_set_state, 0, sizeof(box_set_state));
    box_set_state[boxIndex] = 1;
    serial->write((char*)box_set_state, 6);
    qDebug() << "Opening Box" << (boxIndex + 1);
  }
}

void SerialNode::sendTrajectoryRequest(const QString& path_name)
{
  if (!show_dest_flag) {
    return;
  }

  if (!nh_ || !trajectory_client_) {
    qDebug() << "ROS service not available for path:" << path_name;
    return;
  }

  navigation_msgs::pub_trajectory srv;
  srv.request.sender = "rqt_interact";
  srv.request.path_name = path_name.toStdString();

  if (trajectory_client_.call(srv)) {
    if (srv.response.success) {
      qDebug() << "Successfully sent trajectory request for path:" << path_name;
    } else {
      qDebug() << "Failed to send trajectory request:" << QString::fromStdString(srv.response.message);
    }
  } else {
    qDebug() << "Failed to call trajectory service";
  }
}

} // namespace gbx_rqt_interact