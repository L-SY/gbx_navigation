#ifndef GBX_DELIVERY_RQT_MAIN_WINDOW_H
#define GBX_DELIVERY_RQT_MAIN_WINDOW_H

#include <QLabel>
#include <QMainWindow>
#include <QMap>
#include <QPushButton>
#include <QTimer>
#include <QWidget>
#include <rqt_gui_cpp/plugin.h>
#include "gbx_delivery_rqt/information_hub.h"

namespace Ui {
class MainWindow;
}

namespace gbx_delivery_rqt {

class MainWindow : public rqt_gui_cpp::Plugin {
  Q_OBJECT

public:
  MainWindow();
  virtual ~MainWindow();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

signals:
  void boxOpened(int boxId);
  void boxClosed(int boxId);
  void destinationSelected(int destination);

private:
  enum Mode {
    NONE,
    DELIVERY,
    PICKUP,
    RETURN_BOX
  };

  enum Page {
    MAIN_PAGE,
    DELIVERY_PHONE_PAGE,
    PICKUP_PHONE_PAGE,
    BOX_SELECTION_PAGE,
    BOX_OPEN_PAGE,
    DESTINATION_PAGE,
    DOOR_CLOSED_PAGE,
    ARRIVAL_PAGE,
    QR_CODE_PAGE
  };

  Ui::MainWindow* ui;
  QMainWindow* widget_;
  QTimer* returnTimer;
  Mode currentMode;
  Page currentPage;
  int selectedCabinetId;
  QString lastPhoneNumber;
  QMap<QString, int> phoneNumberToCabinet; // 手机号后四位到柜子ID的映射
  QMap<int, QPushButton*> cabinetButtons;  // 柜子ID到按钮的映射
  QVector<QLabel*> destinationMarkers;     // 目的地标记
  gbx_rqt_interact::InformationHub* infoHub;

  // 新增成员变量
  QMap<int, QPushButton*> destinationButtons; // 目的地按钮映射
  int selectedDestination;                    // 当前选择的目的地
  QLabel* destinationInfoLabel;               // 目的地信息标签
  QPushButton* destinationConfirmButton;      // 目的地确认按钮

  void setupInfoHub();
  void setupUi();
  void setupBackground();
  void setupConnections();
  void setupCabinetButtons();
  void setupQRCodePage();
  void setupDestinationPage();

  void updateCabinetAvailability(const std::vector<navigation_msgs::CabinetContent>& contents);
  void updateCabinetButtonStyle(QPushButton* button, bool isEmpty, bool isForDeliveryOrPickup);
  void updateDoorStatus(int boxId, bool isOpen);

  void handleBoxSelection(int box);
  void handlePhoneNumberSubmit();
  void handleDoorStateUpdate();
  void handleContentsUpdate();
  void handleTrajectoryResult(bool success, const QString& message);
  void handleNavigationArrival(bool arrived);
  void handleDestinationSelect(int destination);
  void confirmDestinationSelection(int destination); // 新增函数

  void switchToMainPage();
  void switchToDeliveryMode();
  void switchToPickupMode();
  void switchToReturnBoxMode();
  void switchToNextPage();
  void switchToBoxSelection();
  void switchToBoxOpen();
  void switchToDestination();
  void switchToDoorClosed();
  void switchToQRCodePage();

  bool validatePhoneNumber(const QString& phone, bool isShortPhone);
  bool isBoxForDeliveryOrPickup(int boxId);
  void showErrorMessage(const QString& message);
  void startWaitForObjectDetection();
  void handleTimeout();

  bool eventFilter(QObject* obj, QEvent* event) override;
};

} // namespace gbx_delivery_rqt

#endif // GBX_DELIVERY_RQT_MAIN_WINDOW_H
