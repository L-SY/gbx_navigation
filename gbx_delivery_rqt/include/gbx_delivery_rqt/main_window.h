#pragma once

#include "gbx_delivery_rqt/information_hub.h"
#include <QDebug>
#include <QLabel>
#include <QMainWindow>
#include <QPainter>
#include <QPixmap>
#include <QPoint>
#include <QPushButton>
#include <QStackedWidget>
#include <QTimer>
#include <QVector>
#include <QtSvg/QSvgRenderer>
#include <rqt_gui_cpp/plugin.h>

namespace Ui {
class MainWindow;
}

namespace gbx_delivery_rqt {

class MainWindow : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  MainWindow();
  virtual ~MainWindow();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings);

protected:
  bool eventFilter(QObject* obj, QEvent* event) override;

private slots:
  // 页面切换相关槽函数
  void switchToPickupMode();    // 切换到取件模式
  void switchToDeliveryMode();  // 切换到寄件模式
  void switchToNextPage();      // 切换到下一页
  void switchToMainPage();      // 返回主页
  void switchToBoxSelection();  // 切换到选择箱子页面
  void switchToDestination();   // 切换到选择目的地页面
  void switchToBoxOpen();       // 切换到箱门打开提示页面
  void switchToDoorClosed();    // 切换到关门感谢页面

  // 业务逻辑相关槽函数
  void handlePhoneNumberSubmit();   // 处理手机号提交
  void handleBoxSelection(int box); // 处理箱子选择
  void handleDestinationSelect(int destination); // 处理目的地选择
  void handleDoorStateChange(bool isOpen, int boxId); // 处理箱门状态变化
  void handleTimeout(); // 处理定时器超时

signals:
  void boxOpened(int boxId);    // 箱门打开信号
  void boxClosed(int boxId);    // 箱门关闭信号
  void destinationSelected(int destination); // 目的地选择信号

private:
  enum PageIndex {
    MAIN_PAGE = 0,          // 主页(取/寄选择)
    DELIVERY_PHONE_PAGE,    // 寄件手机号输入
    BOX_SELECTION_PAGE,     // 选择箱子
    DESTINATION_PAGE,       // 选择目的地
    PICKUP_PHONE_PAGE,      // 取件手机号输入
    BOX_OPEN_PAGE,         // 箱门打开提示
    DOOR_CLOSED_PAGE       // 关门感谢页
  };

  enum DeliveryMode {
    NONE,
    PICKUP,
    DELIVERY
  };

  Ui::MainWindow* ui;
  QMainWindow* widget_;
  gbx_rqt_interact::InformationHub* infoHub;
  QTimer* returnTimer;      // 用于自动返回主页的定时器

  void setupInfoHub();
  void updateBoxAvailability(const std::vector<navigation_msgs::CabinetContent>& contents);
  void handleDoorStateUpdate();
  void handleContentsUpdate();
  void handleTrajectoryResult(bool success, const QString& message);

  QMap<int, QPushButton*> boxButtons;  // To store box button references

  void setupUi();           // 设置UI
  void setupBackground();   // 设置背景
  void setupConnections();  // 设置信号槽连接
  void initializePages();   // 初始化所有页面
  void setupBoxButtons();   // 设置箱子按钮
  void updateDoorStatus(int boxId, bool isOpen); // 更新箱门状态显示

  bool validatePhoneNumber(const QString& phone, bool isShortPhone = false);
  void showErrorMessage(const QString& message);

  PageIndex currentPage;    // 当前页面索引
  DeliveryMode currentMode; // 当前模式（取件/寄件）
  QString lastPhoneNumber;  // 存储寄件人手机号
  int selectedBoxId;        // 当前选中的箱子ID

  void setupDestinationPage();  // 设置目的地选择页面
  void createDestinationMarkers();  // 创建目的地标记
  QLabel* createStarMarker(const QPoint& pos);  // 创建星形标记

  QVector<QLabel*> destinationMarkers;  // 存储目的地标记
  QVector<QPoint> destinationPositions;  // 存储目的地位置
  static const int NUM_DESTINATIONS = 7;  // 目的地数量
};

} // namespace gbx_delivery_rqt