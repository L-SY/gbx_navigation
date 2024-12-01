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

public slots:
  void ChangeBox();
  void ChangeDest();
  void OpenBox1();
  void OpenBox2();
  void OpenBox3();
  void OpenBox4();
  void OpenBox5();
  void OpenBox6();
  void UpdateUI();
  void SendTrajectoryRequest(const QString& path_name);

private:
  Ui::MainWindow* mainWindow_ui;
  QSerialPort* serial;
  QStringList labels = {"A", "B", "C", "D", "E", "F"};

  bool show_box_flag;
  bool show_dest_flag;
  bool send_set_flag;
  bool button_update_flag;

  uint8_t box_fdb_state[8];
  uint8_t box_set_state[8];

  ros::NodeHandle* nh_;
  ros::ServiceClient trajectory_client_;
  ros::Publisher door_state_pub_;
  ros::Subscriber cabinet_content_sub_;

  struct DoorState {
    bool is_open;
    ros::Time last_changed;
    std::string status;
  };
  std::vector<DoorState> previous_door_states_;
  std::vector<navigation_msgs::CabinetContent> current_contents_;

  void readSerialData();
  void publishDoorStates();
  void cabinetContentCallback(const navigation_msgs::CabinetContentArray::ConstPtr& msg);
};

} // namespace gbx_rqt_interact