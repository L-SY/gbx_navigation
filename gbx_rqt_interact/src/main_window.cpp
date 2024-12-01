//
// Created by lsy on 24-12-1.
//

#include "gbx_rqt_interact/main_window.h"
#include "ui_main_window.h"

#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <QDir>
#include <ros/package.h>

namespace gbx_rqt_interact {

MainWindow::MainWindow()
    : rqt_gui_cpp::Plugin()
      , ui(new Ui::MainWindow)
      , widget_(nullptr)
      , serial_node(new SerialNode())
{
  setObjectName("GBXInteractPlugin");
}

MainWindow::~MainWindow()
{
  delete ui;
  if(widget_) {
    delete widget_;
  }
}

void MainWindow::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QMainWindow();

  ui->setupUi(widget_);

  setupBackground();

  serial_node->Init(ui);

  context.addWidget(widget_);
}

void MainWindow::setupBackground()
{
  // 获取资源路径
  QString pkg_path = QString::fromStdString(ros::package::getPath("gbx_rqt_interact"));
  QString image_path = pkg_path + "/resource/images/blue_four.png";

  // 设置背景
  QPixmap backgroundPixmap(image_path);
  if (!backgroundPixmap.isNull()) {
    QBrush brush;
    brush.setTexture(backgroundPixmap);
    QPalette palette;
    palette.setBrush(QPalette::Window, brush);
    widget_->setPalette(palette);
    widget_->setAutoFillBackground(true);
  } else {
    ROS_WARN("Could not load background image: %s", image_path.toStdString().c_str());
  }
}

void MainWindow::shutdownPlugin()
{
  if (serial_node) {
    delete serial_node;
    serial_node = nullptr;
  }
}

void MainWindow::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // 保存插件设置
  // 例如:
  // instance_settings.setValue("serial_port", "/dev/ttyACM0");
}

void MainWindow::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // 恢复插件设置
  // 例如:
  // QString port = instance_settings.value("serial_port", "/dev/ttyACM0").toString();
  // serial_node->setSerialPort(port);
}

} // namespace gbx_rqt_interact

PLUGINLIB_EXPORT_CLASS(gbx_rqt_interact::MainWindow, rqt_gui_cpp::Plugin)
