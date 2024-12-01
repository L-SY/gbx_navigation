#pragma once

#include <QThread>
#include <QSerialPort>
#include <QObject>
#include <QPushButton>
#include <QString>
#include <ros/ros.h>
#include <navigation_msgs/pub_trajectory.h>
#include <navigation_msgs/CabinetDoorArray.h>
#include <navigation_msgs/CabinetContentArray.h>
#include <QTimer>

namespace Ui {
class MainWindow;
}

namespace gbx_rqt_interact {

class SerialNode : public QThread
{
  Q_OBJECT
public:
  explicit SerialNode(QObject *parent = nullptr);
  virtual ~SerialNode();

  void Init(Ui::MainWindow* _mainWindow_ui);
  void DeviceInit();
  void UiInit();
  void ROSInit();

protected:
  void run() override;

private slots:
  void handleButtonClick(int buttonIndex);
  void handleBoxButtonClick();
  void handleDestButtonClick();
  void UpdateUI();
  void processSerialData();

signals:
  void requestUIUpdate();

private:
  Ui::MainWindow* mainWindow_ui;
  QSerialPort* serial;
  QStringList labels = {"A", "B", "C", "D", "E", "F"};
  QTimer* updateTimer;

  bool show_box_flag;
  bool show_dest_flag;
  bool send_set_flag;
  bool button_update_flag;

  uint8_t box_fdb_state[8];
  uint8_t box_set_state[8];
  uint8_t last_box_state[8];
  std::vector<navigation_msgs::CabinetContent> last_contents_;

  ros::NodeHandle* nh_;
  ros::ServiceClient trajectory_client_;
  ros::Publisher door_state_pub_;
  ros::Subscriber cabinet_content_sub_;

  std::vector<navigation_msgs::CabinetContent> current_contents_;

  void readSerialData();
  void publishDoorStates();
  void cabinetContentCallback(const navigation_msgs::CabinetContentArray::ConstPtr& msg);
  void updateButton(int index);
  void sendBoxCommand(int boxIndex);
  void sendTrajectoryRequest(const QString& path_name);
  bool hasStateChanged() const;
  bool hasContentsChanged() const;
};

} // namespace gbx_rqt_interact