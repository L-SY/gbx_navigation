//
// Created by lsy on 24-12-1.
//

#pragma once

#include <rqt_gui_cpp/plugin.h>
#include <QMainWindow>
#include <QPixmap>
#include <QBrush>
#include <QPalette>
#include "serial_node.h"

namespace Ui {
class MainWindow;
}

namespace gbx_rqt_interact {

class MainWindow : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  MainWindow();
  virtual ~MainWindow();

  // Plugin 接口实现
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private:
  Ui::MainWindow* ui;
  QMainWindow* widget_;
  SerialNode* serial_node;

  void setupUi();
  void setupBackground();
  void connectSignals();
};

} // namespace gbx_rqt_interact

