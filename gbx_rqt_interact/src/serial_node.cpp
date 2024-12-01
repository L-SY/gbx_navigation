//
// Created by lsy on 24-12-1.
//

#include "gbx_rqt_interact/serial_node.h"
#include "ui_main_window.h"
#include <QTime>
#include <QDebug>

namespace gbx_rqt_interact {

SerialNode::SerialNode(QObject *parent)
    : QThread(parent), show_box_flag(false), show_dest_flag(false),
      send_set_flag(false), button_update_flag(false), serial(nullptr)
{
  memset(box_fdb_state, 0, sizeof(box_fdb_state));
  memset(box_set_state, 0, sizeof(box_set_state));
}

SerialNode::~SerialNode() {
  if (serial && serial->isOpen()) {
    serial->close();
  }
  delete serial;
}

void SerialNode::Init(Ui::MainWindow* _mainWindow_ui)
{
  mainWindow_ui = _mainWindow_ui;

  serial = new QSerialPort();
  DeviceInit();
  UiInit();

  // 连接信号槽，确保UI更新在主线程中进行
  connect(this, &SerialNode::requestUIUpdate, this, &SerialNode::UpdateUI, Qt::QueuedConnection);

  // 启动线程
  start();
}

void SerialNode::DeviceInit()
{
  serial->setPortName("/dev/ttyACM0");
  serial->setBaudRate(QSerialPort::Baud115200);
  serial->setDataBits(QSerialPort::Data8);
  serial->setParity(QSerialPort::NoParity);
  serial->setStopBits(QSerialPort::OneStop);
  serial->setFlowControl(QSerialPort::NoFlowControl);

  if (serial->open(QIODevice::ReadWrite)) {
    serial->setDataTerminalReady(true);
  } else {
    qDebug() << "Failed to open serial port!";
  }
}

void SerialNode::UiInit()
{
  connect(mainWindow_ui->boxButton, &QPushButton::clicked, this, &SerialNode::ChangeBox, Qt::QueuedConnection);
  connect(mainWindow_ui->destButton, &QPushButton::clicked, this, &SerialNode::ChangeDest, Qt::QueuedConnection);
  connect(mainWindow_ui->B1, &QPushButton::clicked, this, &SerialNode::OpenBox1, Qt::QueuedConnection);
  connect(mainWindow_ui->B2, &QPushButton::clicked, this, &SerialNode::OpenBox2, Qt::QueuedConnection);
  connect(mainWindow_ui->B3, &QPushButton::clicked, this, &SerialNode::OpenBox3, Qt::QueuedConnection);
  connect(mainWindow_ui->B4, &QPushButton::clicked, this, &SerialNode::OpenBox4, Qt::QueuedConnection);
  connect(mainWindow_ui->B5, &QPushButton::clicked, this, &SerialNode::OpenBox5, Qt::QueuedConnection);
  connect(mainWindow_ui->B6, &QPushButton::clicked, this, &SerialNode::OpenBox6, Qt::QueuedConnection);
}

void SerialNode::run()
{
  while (true) {
    readSerialData();
    msleep(100);
  }
}

void SerialNode::readSerialData()
{
  if (serial->bytesAvailable() >= 8) {
    serial->read((char*)box_fdb_state, 8);
    emit requestUIUpdate(); // 请求主线程更新UI
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

void SerialNode::ChangeBox()
{
  show_box_flag = true;
  show_dest_flag = false;
  emit requestUIUpdate();
}

void SerialNode::ChangeDest()
{
  show_box_flag = false;
  show_dest_flag = true;
  emit requestUIUpdate();
}

void SerialNode::OpenBox1() { box_set_state[0] = 1; serial->write((char*)box_set_state, 6); box_set_state[0] = 0; }
void SerialNode::OpenBox2() { box_set_state[1] = 1; serial->write((char*)box_set_state, 6); box_set_state[1] = 0; }
void SerialNode::OpenBox3() { box_set_state[2] = 1; serial->write((char*)box_set_state, 6); box_set_state[2] = 0; }
void SerialNode::OpenBox4() { box_set_state[3] = 1; serial->write((char*)box_set_state, 6); box_set_state[3] = 0; }
void SerialNode::OpenBox5() { box_set_state[4] = 1; serial->write((char*)box_set_state, 6); box_set_state[4] = 0; }
void SerialNode::OpenBox6() { box_set_state[5] = 1; serial->write((char*)box_set_state, 6); box_set_state[5] = 0; }

} // namespace gbx_rqt_interact