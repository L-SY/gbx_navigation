#include "gbx_rqt_interact/serial_node.h"
#include "ui_main_window.h"
#include <QDebug>
#include <QMessageBox>
#include <QTime>
#include <QDateTime>
#include <algorithm>

namespace gbx_rqt_interact {

const QString SerialNode::DEFAULT_PORT_NAME = "/dev/ttyACM0";

SerialNode::SerialNode(QObject *parent)
    : QObject(parent)
      , mainWindow_ui(nullptr)
      , serial(nullptr)
      , rosTimer(new QTimer(this))
      , buttonMapper(new QSignalMapper(this))
      , show_box_flag(false)
      , show_dest_flag(false)
      , send_set_flag(false)
      , button_update_flag(false)
      , button_dirty_flags_(6, false)
      , nh_(nullptr)
      , reconnect_attempts_(0)
      , reconnectTimer(new QTimer(this))
{
  // 初始化状态数组
  memset(box_fdb_state, 0, sizeof(box_fdb_state));
  memset(box_set_state, 0, sizeof(box_set_state));
  memset(last_box_state, 0, sizeof(last_box_state));

  // 设置ROS定时器频率为100Hz
  rosTimer->setInterval(10);
  connect(rosTimer, &QTimer::timeout, this, &SerialNode::processROSData);

  // 设置重连定时器
  reconnectTimer->setInterval(SERIAL_RECONNECT_TIMEOUT);
  reconnectTimer->setSingleShot(true);
  connect(reconnectTimer, &QTimer::timeout, this, &SerialNode::tryReconnectSerial);

  // 预分配门状态消息
  door_states_msg_.doors.resize(6);
  for (int i = 0; i < 6; ++i) {
    door_states_msg_.doors[i].id = "door_" + std::to_string(i + 1);
  }
}

SerialNode::~SerialNode()
{
  rosTimer->stop();
  reconnectTimer->stop();

  if (serial && serial->isOpen()) {
    serial->close();
  }
  delete serial;
  delete nh_;
  delete buttonMapper;
}

void SerialNode::Init(Ui::MainWindow* _mainWindow_ui)
{
  mainWindow_ui = _mainWindow_ui;
  DeviceInit();

  try {
    ROSInit();
  } catch (const std::exception& e) {
    logMessage("ROS initialization failed: " + QString(e.what()), true);
  }

  UiInit();
  connect(this, &SerialNode::requestUIUpdate, this, &SerialNode::UpdateUI, Qt::QueuedConnection);
  start();
}

void SerialNode::DeviceInit()
{
  if (!serial) {
    serial = new QSerialPort(this);
  }

  current_serial_port_ = DEFAULT_PORT_NAME;
  serial->setPortName(current_serial_port_);
  serial->setBaudRate(DEFAULT_BAUD_RATE);
  serial->setDataBits(QSerialPort::Data8);
  serial->setParity(QSerialPort::NoParity);
  serial->setStopBits(QSerialPort::OneStop);
  serial->setFlowControl(QSerialPort::NoFlowControl);

  connect(serial, &QSerialPort::readyRead, this, &SerialNode::onSerialDataReady);
  connect(serial, &QSerialPort::errorOccurred, this, &SerialNode::onSerialError);

  if (!tryReconnectSerial()) {
    logMessage("Initial serial connection failed. Will retry...", true);
    reconnectTimer->start();
  }
}

bool SerialNode::tryReconnectSerial()
{
  if (serial->isOpen()) {
    serial->close();
  }

  if (serial->open(QIODevice::ReadWrite)) {
    serial->setDataTerminalReady(true);
    reconnect_attempts_ = 0;
    logMessage("Serial port opened successfully: " + current_serial_port_);
    return true;
  }

  if (++reconnect_attempts_ < 3) {
    reconnectTimer->start();
  } else {
    logMessage("Failed to open serial port after multiple attempts", true);
  }
  return false;
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
  // 设置按钮信号映射
  QPushButton* buttons[6] = {
      mainWindow_ui->B1, mainWindow_ui->B2, mainWindow_ui->B3,
      mainWindow_ui->B4, mainWindow_ui->B5, mainWindow_ui->B6
  };

  for (int i = 0; i < 6; ++i) {
    connect(buttons[i], SIGNAL(clicked()), buttonMapper, SLOT(map()));
    buttonMapper->setMapping(buttons[i], i);
  }
  connect(buttonMapper, SIGNAL(mapped(int)), this, SLOT(handleButtonClick(int)));

  // 连接模式切换按钮
  connect(mainWindow_ui->boxButton, &QPushButton::clicked,
          this, &SerialNode::handleBoxButtonClick, Qt::QueuedConnection);
  connect(mainWindow_ui->destButton, &QPushButton::clicked,
          this, &SerialNode::handleDestButtonClick, Qt::QueuedConnection);
}

void SerialNode::start()
{
  rosTimer->start();
}

void SerialNode::processROSData()
{
  if (nh_) {
    ros::spinOnce();
  }
}

void SerialNode::onSerialDataReady()
{
  while (serial->bytesAvailable() >= 8) {
    QByteArray data = serial->read(8);
    if (data.size() == 8) {
      uint8_t new_state[8];
      memcpy(new_state, data.constData(), 8);

      changed_states_.clear();
      bool hasChanges = false;

      for (int i = 0; i < 8; ++i) {
        if (new_state[i] != box_fdb_state[i]) {
          changed_states_[i] = new_state[i];
          if (i < 6) {
            button_dirty_flags_[i] = true;
          }
          hasChanges = true;
        }
      }

      if (hasChanges) {
        memcpy(box_fdb_state, new_state, 8);
        emit requestUIUpdate();
        publishDoorStates();
      }
    }
  }
}

void SerialNode::onSerialError(QSerialPort::SerialPortError error)
{
  if (error == QSerialPort::NoError) {
    return;
  }

  QString errorMessage = "Serial port error: " + QString::number(error);
  logMessage(errorMessage, true);

  if (error != QSerialPort::NotOpenError) {
    reconnectTimer->start();
  }
}

void SerialNode::UpdateUI()
{
  if (show_box_flag != button_update_flag) {
    // UI模式改变，更新所有按钮
    for (int i = 0; i < 6; ++i) {
      button_dirty_flags_[i] = true;
    }
    button_update_flag = show_box_flag;
  }

  // 只更新需要更新的按钮
  for (int i = 0; i < 6; ++i) {
    if (button_dirty_flags_[i]) {
      updateButton(i);
      button_dirty_flags_[i] = false;
    }
  }
}

void SerialNode::updateButton(int index)
{
  QPushButton* buttons[6] = {
      mainWindow_ui->B1, mainWindow_ui->B2, mainWindow_ui->B3,
      mainWindow_ui->B4, mainWindow_ui->B5, mainWindow_ui->B6
  };

  if (show_box_flag) {
    QString buttonText;
    std::string cabinet_id = "cabinet_" + std::to_string(index + 1);

    auto it = std::find_if(current_contents_.begin(), current_contents_.end(),
                           [&](const navigation_msgs::CabinetContent& content) {
                             return content.cabinet_id == cabinet_id;
                           });

    buttons[index]->setStyleSheet(box_fdb_state[index+1] == 1
                                      ? "color: #A367CA;"
                                      : "color: #091648;");

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

void SerialNode::publishDoorStates()
{
  if (changed_states_.isEmpty()) {
    return;
  }

  door_states_msg_.header.stamp = ros::Time::now();

  for (auto it = changed_states_.constBegin(); it != changed_states_.constEnd(); ++it) {
    int door_idx = it.key();
    if (door_idx < 6) {
      auto& door = door_states_msg_.doors[door_idx];
      door.is_open = (it.value() == 1);
      door.status = door.is_open ? "open" : "closed";
      door.last_changed = ros::Time::now();
    }
  }

  door_state_pub_.publish(door_states_msg_);
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

void SerialNode::cabinetContentCallback(const navigation_msgs::CabinetContentArray::ConstPtr& msg)
{
  current_contents_ = msg->cabinets;

  // 检查内容是否发生变化
  if (current_contents_.size() != last_contents_.size()) {
    for (int i = 0; i < 6; ++i) {
      button_dirty_flags_[i] = true;
    }
    emit requestUIUpdate();
    return;
  }

  for (size_t i = 0; i < current_contents_.size(); ++i) {
    if (i < last_contents_.size() &&
        current_contents_[i].box.box_id != last_contents_[i].box.box_id) {
      int buttonIndex = std::stoi(current_contents_[i].cabinet_id.substr(8)) - 1;
      if (buttonIndex >= 0 && buttonIndex < 6) {
        button_dirty_flags_[buttonIndex] = true;
      }
    }
  }

  if (std::any_of(button_dirty_flags_.begin(), button_dirty_flags_.end(),
                  [](bool flag) { return flag; })) {
    emit requestUIUpdate();
  }
}

void SerialNode::sendBoxCommand(int boxIndex)
{
  if (!serial || !serial->isOpen()) {
    logMessage("Cannot send command: Serial port not open", true);
    return;
  }

  memset(box_set_state, 0, sizeof(box_set_state));
  box_set_state[boxIndex] = 1;

  QByteArray data((char*)box_set_state, 6);
  qint64 bytesWritten = serial->write(data);

  if (bytesWritten != 6) {
    logMessage("Failed to write complete command to serial port", true);
    return;
  }

  if (!serial->waitForBytesWritten(100)) {
    logMessage("Timeout waiting for command to be written", true);
    return;
  }

  logMessage(QString("Sent command to open Box %1").arg(boxIndex + 1));
}

void SerialNode::sendTrajectoryRequest(const QString& path_name)
{
  if (!show_dest_flag) {
    return;
  }

  if (!nh_ || !trajectory_client_) {
    logMessage("ROS service not available for path: " + path_name, true);
    return;
  }

  navigation_msgs::pub_trajectory srv;
  srv.request.sender = "rqt_interact";
  srv.request.path_name = path_name.toStdString();

  try {
    if (trajectory_client_.call(srv)) {
      if (srv.response.success) {
        logMessage("Successfully sent trajectory request for path: " + path_name);
      } else {
        logMessage("Failed to send trajectory request: " +
                       QString::fromStdString(srv.response.message), true);
      }
    } else {
      logMessage("Failed to call trajectory service", true);
    }
  } catch (const ros::Exception& e) {
    logMessage("ROS error while sending trajectory request: " +
                   QString::fromStdString(e.what()), true);
  }
}

void SerialNode::logMessage(const QString& message, bool isError)
{
  QString logMsg = QTime::currentTime().toString("hh:mm:ss.zzz") + ": " + message;
  if (isError) {
    qWarning() << logMsg;
  } else {
    qDebug() << logMsg;
  }
}

} // namespace gbx_rqt_interact

#include "serial_node.moc"