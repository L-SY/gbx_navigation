#pragma once

#include "gbx_delivery_rqt/information_hub.h"
#include "navigation_msgs/IndoorDeliveryOrder.h"
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

class MainWindow : public rqt_gui_cpp::Plugin {
  Q_OBJECT

public:
  // 构造和析构
  MainWindow();
  virtual ~MainWindow();

  // RQT插件接口方法
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings);

protected:
  bool eventFilter(QObject* obj, QEvent* event) override;

private:
  // 枚举定义
  enum PageIndex {
    MAIN_PAGE = 0,         // 主页(取/寄选择)
    DELIVERY_PHONE_PAGE,   // 寄件手机号输入
    BOX_SELECTION_PAGE,    // 选择箱子
    DESTINATION_PAGE,      // 选择目的地
    PICKUP_PHONE_PAGE,     // 取件手机号输入
    BOX_OPEN_PAGE,        // 箱门打开提示
    DOOR_CLOSED_PAGE,      // 关门感谢页
    ARRIVAL_PAGE
  };

  enum DeliveryMode {
    NONE,
    PICKUP,
    DELIVERY
  };

  // UI初始化方法
  void setupUi();                // 设置主UI
  void setupBackground();        // 设置背景
  void setupConnections();       // 设置信号槽连接
  void setupCabinetButtons();    // 设置箱子按钮
  void setupDestinationPage();   // 设置目的地选择页面
  void setupInfoHub();           // 设置信息中心

  // UI更新方法
  void updateCabinetButtonStyle(QPushButton* button, bool isCabinetEmpty);
  void updateDoorStatus(int boxId, bool isOpen);
  void updateCabinetAvailability(const std::vector<navigation_msgs::CabinetContent>& contents);

  // 业务逻辑处理方法
  bool validatePhoneNumber(const QString& phone, bool isShortPhone = false);
  void showErrorMessage(const QString& message);
  void startWaitForObjectDetection();

private slots:
  // 页面导航相关槽
  void switchToPickupMode();     // 切换到取件模式
  void switchToDeliveryMode();   // 切换到寄件模式
  void switchToNextPage();       // 切换到下一页
  void switchToMainPage();       // 返回主页
  void switchToBoxSelection();   // 切换到选择箱子页面
  void switchToDestination();    // 切换到选择目的地页面
  void switchToBoxOpen();        // 切换到箱门打开提示页面
  void switchToDoorClosed();     // 切换到关门感谢页面

  // 事件处理槽
  void handlePhoneNumberSubmit();    // 处理手机号提交
  void handleBoxSelection(int box);  // 处理箱子选择
  void handleDestinationSelect(int destination); // 处理目的地选择
  void handleTimeout();              // 处理定时器超时
  void handleDoorStateUpdate();      // 处理门状态更新
  void handleContentsUpdate();       // 处理内容更新
  void handleTrajectoryResult(bool success, const QString& message);
  void handleNavigationArrival(bool arrived);

signals:
  void boxOpened(int boxId);            // 箱门打开信号
  void boxClosed(int boxId);            // 箱门关闭信号
  void destinationSelected(int destination); // 目的地选择信号

private:
  // UI组件
  Ui::MainWindow* ui;
  QMainWindow* widget_;
  QMap<int, QPushButton*> cabinetButtons;    // 存储箱子按钮引用
  QMap<QString, int> phoneNumberToCabinet;
  QVector<QLabel*> destinationMarkers;       // 存储目的地标记

  // 核心组件
  gbx_rqt_interact::InformationHub* infoHub;
  QTimer* returnTimer;                       // 自动返回主页定时器

  // 状态变量
  PageIndex currentPage;                     // 当前页面索引
  DeliveryMode currentMode;                  // 当前模式（取件/寄件）
  QString lastPhoneNumber;                   // 存储寄件人手机号
  int selectedCabinetId;                     // 当前选中的柜子ID
};

} // namespace gbx_delivery_rqt