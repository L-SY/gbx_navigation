//
// Created by lsy on 24-12-3.
//

#include "gbx_delivery_rqt/information_hub.h"
#include <QDebug>

namespace gbx_rqt_interact {

InformationHub::InformationHub(QObject *parent)
    : QThread(parent),
      serial(nullptr),
      updateTimer(nullptr),
      nh_(nullptr)
{
  memset(box_fdb_state, 0, sizeof(box_fdb_state));
  memset(box_set_state, 0, sizeof(box_set_state));
  memset(last_box_state, 0, sizeof(last_box_state));
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
  } catch (const std::exception& e) {
    qDebug() << "ROS initialization failed, but continuing with other functions";
  }

  updateTimer->start();
}

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
    qDebug() << "Serial port opened successfully";
    return true;
  } else {
    qDebug() << "Failed to open serial port!";
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
  cabinet_content_sub_ = nh_->subscribe("/cabinet/contents", 1,
                                        &InformationHub::cabinetContentCallback, this);
}

void InformationHub::run()
{
  while (!isInterruptionRequested()) {
    if (nh_) {
      ros::spinOnce();
    }
    msleep(20); // Reduce thread frequency to 50Hz
  }
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
    if (memcmp(new_state, box_fdb_state, 8) != 0) {
      memcpy(box_fdb_state, new_state, 8);
      emit doorStateChanged();
      publishDoorStates();
    }
  }
}

bool InformationHub::hasStateChanged() const
{
  return memcmp(box_fdb_state, last_box_state, 8) != 0;
}

bool InformationHub::hasContentsChanged() const
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

void InformationHub::publishDoorStates()
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

void InformationHub::cabinetContentCallback(const navigation_msgs::CabinetContentArray::ConstPtr& msg)
{
  current_contents_ = msg->cabinets;
  emit contentStateChanged();
}

void InformationHub::sendBoxCommand(int boxIndex)
{
  if (serial && serial->isOpen()) {
    memset(box_set_state, 0, sizeof(box_set_state));
    box_set_state[boxIndex] = 1;
    serial->write((char*)box_set_state, 6);
    qDebug() << "Opening Box" << (boxIndex + 1);
  }
}

void InformationHub::sendTrajectoryRequest(const QString& path_name)
{
  if (!nh_ || !trajectory_client_) {
    emit trajectoryRequestResult(false, "ROS service not available");
    return;
  }

  navigation_msgs::pub_trajectory srv;
  srv.request.sender = "rqt_interact";
  srv.request.path_name = path_name.toStdString();

  if (trajectory_client_.call(srv)) {
    emit trajectoryRequestResult(srv.response.success,
                                 QString::fromStdString(srv.response.message));
  } else {
    emit trajectoryRequestResult(false, "Failed to call trajectory service");
  }
}

} // namespace gbx_rqt_interact
