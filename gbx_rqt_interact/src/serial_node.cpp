#include "gbx_rqt_interact/serial_node.h"
#include "ui_main_window.h"
#include <QTime>
#include <QDebug>

namespace gbx_rqt_interact {

SerialNode::SerialNode(QObject *parent)
    : QThread(parent),
      mainWindow_ui(nullptr),
      serial(nullptr),
      show_box_flag(false),
      show_dest_flag(false),
      send_set_flag(false),
      button_update_flag(false),
      nh_(nullptr)
{
  memset(box_fdb_state, 0, sizeof(box_fdb_state));
  memset(box_set_state, 0, sizeof(box_set_state));

  previous_door_states_.resize(6);
  for (auto& state : previous_door_states_) {
    state.is_open = false;
    state.last_changed = ros::Time::now();
    state.status = "closed";
  }
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
}

void SerialNode::Init(Ui::MainWindow* _mainWindow_ui)
{
  mainWindow_ui = _mainWindow_ui;

  serial = new QSerialPort();
  DeviceInit();

  try {
    ROSInit();
  } catch (const std::exception& e) {
    qDebug() << "ROS initialization failed, but continuing with other functions";
  }

  UiInit();

  connect(this, &SerialNode::requestUIUpdate, this, &SerialNode::UpdateUI, Qt::QueuedConnection);

  start();
}

void SerialNode::DeviceInit()
{
  if (!serial) {
    serial = new QSerialPort();
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

  trajectory_client_ = nh_->serviceClient<navigation_msgs::pub_trajectory>("/gbx_manual/pub_trajectory");

  door_state_pub_ = nh_->advertise<navigation_msgs::CabinetDoorArray>("/cabinet/door_states", 10);

  cabinet_content_sub_ = nh_->subscribe("/cabinet/contents_update", 10,
                                        &SerialNode::cabinetContentCallback, this);
}

void SerialNode::publishDoorStates() {
  bool states_changed = false;
  navigation_msgs::CabinetDoorArray msg;
  msg.header.stamp = ros::Time::now();

  for (int i = 0; i < 6; ++i) {
    bool current_state = (box_fdb_state[i + 1] == 1);
    std::string current_status = current_state ? "open" : "closed";

    if (current_state != previous_door_states_[i].is_open) {
      previous_door_states_[i].is_open = current_state;
      previous_door_states_[i].last_changed = ros::Time::now();
      previous_door_states_[i].status = current_status;
      states_changed = true;
    }

    navigation_msgs::CabinetDoor door;
    door.id = "door_" + std::to_string(i + 1);
    door.is_open = current_state;
    door.status = current_status;
    door.last_changed = previous_door_states_[i].last_changed;
    msg.doors.push_back(door);
  }

  if (states_changed) {
    door_state_pub_.publish(msg);
  }
}

void SerialNode::cabinetContentCallback(const navigation_msgs::CabinetContentArray::ConstPtr& msg)
{
  bool contents_changed = false;

  if (msg->cabinets.size() != previous_contents_.size()) {
    contents_changed = true;
  } else {
    for (size_t i = 0; i < msg->cabinets.size(); ++i) {
      if (msg->cabinets[i].box.box_id != previous_contents_[i].box.box_id ||
          msg->cabinets[i].status != previous_contents_[i].status) {
        contents_changed = true;
        break;
      }
    }
  }

  if (contents_changed) {
    current_contents_ = msg->cabinets;
    previous_contents_ = current_contents_;
    emit requestUIUpdate();
  }
}

void SerialNode::updateCabinetDisplay()
{
  QPushButton* buttons[6] = {mainWindow_ui->B1, mainWindow_ui->B2, mainWindow_ui->B3,
                             mainWindow_ui->B4, mainWindow_ui->B5, mainWindow_ui->B6};

  if (show_box_flag) {
    for (int i = 0; i < 6; ++i) {
      QString buttonText;
      std::string cabinet_id = "cabinet_" + std::to_string(i + 1);

      auto it = std::find_if(current_contents_.begin(), current_contents_.end(),
                             [&](const navigation_msgs::CabinetContent& content) {
                               return content.cabinet_id == cabinet_id;
                             });

      if (it != current_contents_.end() && !it->box.box_id.empty()) {
        buttonText = QString("Box %1\n%2")
                         .arg(i + 1)
                         .arg(QString::fromStdString(it->box.box_id));
        buttons[i]->setStyleSheet("color: #A367CA;");
      } else {
        buttonText = QString("Box %1\n%2")
                         .arg(i + 1)
                         .arg(box_fdb_state[i+1] == 1 ? "OPEN" : "CLOSE");
        buttons[i]->setStyleSheet("color: #091648;");
      }
      buttons[i]->setText(buttonText);
    }
  }
}

void SerialNode::UiInit()
{
  if(mainWindow_ui->B1) mainWindow_ui->B1->disconnect();
  if(mainWindow_ui->B2) mainWindow_ui->B2->disconnect();
  if(mainWindow_ui->B3) mainWindow_ui->B3->disconnect();
  if(mainWindow_ui->B4) mainWindow_ui->B4->disconnect();
  if(mainWindow_ui->B5) mainWindow_ui->B5->disconnect();
  if(mainWindow_ui->B6) mainWindow_ui->B6->disconnect();

  // 模式切换按钮
  connect(mainWindow_ui->boxButton, &QPushButton::clicked, this, &SerialNode::ChangeBox, Qt::QueuedConnection);
  connect(mainWindow_ui->destButton, &QPushButton::clicked, this, &SerialNode::ChangeDest, Qt::QueuedConnection);

  if (show_box_flag) {
    // Box模式下的连接
    connect(mainWindow_ui->B1, &QPushButton::clicked, this, &SerialNode::OpenBox1, Qt::QueuedConnection);
    connect(mainWindow_ui->B2, &QPushButton::clicked, this, &SerialNode::OpenBox2, Qt::QueuedConnection);
    connect(mainWindow_ui->B3, &QPushButton::clicked, this, &SerialNode::OpenBox3, Qt::QueuedConnection);
    connect(mainWindow_ui->B4, &QPushButton::clicked, this, &SerialNode::OpenBox4, Qt::QueuedConnection);
    connect(mainWindow_ui->B5, &QPushButton::clicked, this, &SerialNode::OpenBox5, Qt::QueuedConnection);
    connect(mainWindow_ui->B6, &QPushButton::clicked, this, &SerialNode::OpenBox6, Qt::QueuedConnection);
  } else if (show_dest_flag) {
    // Dest模式下的连接
    connect(mainWindow_ui->B1, &QPushButton::clicked, this, [this](){ SendTrajectoryRequest("E3_121"); }, Qt::QueuedConnection);
    connect(mainWindow_ui->B2, &QPushButton::clicked, this, [this](){ SendTrajectoryRequest("2F_sl_go"); }, Qt::QueuedConnection);
    connect(mainWindow_ui->B3, &QPushButton::clicked, this, [this](){ SendTrajectoryRequest("2F_sl_back"); }, Qt::QueuedConnection);
    connect(mainWindow_ui->B4, &QPushButton::clicked, this, [this](){ SendTrajectoryRequest("D"); }, Qt::QueuedConnection);
    connect(mainWindow_ui->B5, &QPushButton::clicked, this, [this](){ SendTrajectoryRequest("E"); }, Qt::QueuedConnection);
    connect(mainWindow_ui->B6, &QPushButton::clicked, this, [this](){ SendTrajectoryRequest("F"); }, Qt::QueuedConnection);
  }
}

void SerialNode::run()
{
  while (!isInterruptionRequested()) {
    if (nh_) {
      ros::spinOnce();
      publishDoorStates();
    }
    readSerialData();
    msleep(50);
  }
}

void SerialNode::readSerialData()
{
  if (serial && serial->isOpen() && serial->bytesAvailable() >= 8) {
    serial->read((char*)box_fdb_state, 8);
    emit requestUIUpdate();
  }
}

void SerialNode::UpdateUI()
{
  QPushButton* buttons[6] = {mainWindow_ui->B1, mainWindow_ui->B2, mainWindow_ui->B3,
                             mainWindow_ui->B4, mainWindow_ui->B5, mainWindow_ui->B6};
  if (show_box_flag) {
    for (int i = 0; i < 6; ++i) {
      if (box_fdb_state[i+1] == 1) {
        buttons[i]->setStyleSheet("color: #A367CA;");
        buttons[i]->setText(QString("Box %1 OPEN").arg(i + 1));
      } else {
        int j = i + 1;
        buttons[i]->setStyleSheet("color: #091648;");
        buttons[i]->setText(QString("Box %1 CLOSE").arg(j));
      }
    }
  } else {
    for (int i = 0; i < 6; ++i) {
      buttons[i]->setStyleSheet("color: rgb(0,0,0)");
      buttons[i]->setText(labels[i]);
    }
  }
}

void SerialNode::SendTrajectoryRequest(const QString& path_name)
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

void SerialNode::ChangeBox()
{
  show_box_flag = true;
  show_dest_flag = false;
  UiInit();  // 重新连接对应的槽函数
  emit requestUIUpdate();
}

void SerialNode::ChangeDest()
{
  show_box_flag = false;
  show_dest_flag = true;
  UiInit();  // 重新连接对应的槽函数
  emit requestUIUpdate();
}

void SerialNode::OpenBox1()
{
  if (serial && serial->isOpen()) {
    memset(box_set_state, 0, sizeof(box_set_state));
    box_set_state[0] = 1;
    serial->write((char*)box_set_state, 6);
    qDebug() << "Opening Box 1";
  } else {
    qDebug() << "Serial port not available for Box 1";
  }
}

void SerialNode::OpenBox2()
{
  if (serial && serial->isOpen()) {
    memset(box_set_state, 0, sizeof(box_set_state));
    box_set_state[1] = 1;
    serial->write((char*)box_set_state, 6);
    qDebug() << "Opening Box 2";
  } else {
    qDebug() << "Serial port not available for Box 2";
  }
}

void SerialNode::OpenBox3()
{
  if (serial && serial->isOpen()) {
    memset(box_set_state, 0, sizeof(box_set_state));
    box_set_state[2] = 1;
    serial->write((char*)box_set_state, 6);
    qDebug() << "Opening Box 3";
  } else {
    qDebug() << "Serial port not available for Box 3";
  }
}

void SerialNode::OpenBox4()
{
  if (serial && serial->isOpen()) {
    memset(box_set_state, 0, sizeof(box_set_state));
    box_set_state[3] = 1;
    serial->write((char*)box_set_state, 6);
    qDebug() << "Opening Box 4";
  } else {
    qDebug() << "Serial port not available for Box 4";
  }
}

void SerialNode::OpenBox5()
{
  if (serial && serial->isOpen()) {
    memset(box_set_state, 0, sizeof(box_set_state));
    box_set_state[4] = 1;
    serial->write((char*)box_set_state, 6);
    qDebug() << "Opening Box 5";
  } else {
    qDebug() << "Serial port not available for Box 5";
  }
}

void SerialNode::OpenBox6()
{
  if (serial && serial->isOpen()) {
    memset(box_set_state, 0, sizeof(box_set_state));
    box_set_state[5] = 1;
    serial->write((char*)box_set_state, 6);
    qDebug() << "Opening Box 6";
  } else {
    qDebug() << "Serial port not available for Box 6";
  }
}

} // namespace gbx_rqt_interact