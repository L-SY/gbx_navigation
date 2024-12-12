//
// Created by lsy on 24-12-3.
//

#pragma once

#include <QObject>
#include <QSerialPort>
#include <QString>
#include <QThread>
#include <QTimer>
#include <QtSerialPort/QSerialPort>
#include <navigation_msgs/IndoorDeliveryOrder.h>
#include <navigation_msgs/OutputDelivery.h>
#include <navigation_msgs/CabinetContentArray.h>
#include <navigation_msgs/CabinetDoorArray.h>
#include <navigation_msgs/pub_trajectory.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseActionResult.h>

namespace gbx_rqt_interact {

class InformationHub : public QThread
{
  Q_OBJECT
public:
  explicit InformationHub(QObject *parent = nullptr);
  virtual ~InformationHub();

  void init();
  bool openSerialPort(const QString& portName = "/dev/ttyACM0");
  void closeSerialPort();
  bool isSerialPortOpen() const;
  void publishIndoorDeliveryOrder(
    const std::string& carNumber,
    const std::string& rfid,
    const std::string& converted_rfid,
    const std::string& owner,
    const std::string& area,
    const std::string& orderNumber,
    const std::string& receiverPhone,
    const std::string& receiverName,
    const std::string& senderName);
  std::vector<navigation_msgs::CabinetContent> getCabinetInfo(){return current_contents_;}
  void publishOutputDelivery(
      const std::string& owner,
      const std::string& rfid,
      const std::string& converted_rfid,
      const std::string& receiverPhone);

signals:
  void doorStateChanged();
  void contentStateChanged();
  void trajectoryRequestResult(bool success, const QString& message);
  void navigationArrived(bool arrived);

private slots:
  void processSerialData();
  void handleNavigationArrived(bool arrived);

protected:
  void run() override;

private:
  // Serial communication
  QSerialPort* serial;
  QTimer* updateTimer;
  uint8_t door_state[8];
  uint8_t door_set_state[8];
  uint8_t last_door_state[8];

  // ROS related
  ros::NodeHandle* nh_;
  ros::ServiceClient trajectory_client_;
  ros::Publisher door_state_pub_, indoor_delivery_pub_, output_delivery_pub_;
  ros::Subscriber cabinet_content_sub_;
  std::vector<navigation_msgs::CabinetContent> current_contents_;
  std::vector<navigation_msgs::CabinetContent> last_contents_;
  ros::Subscriber navigation_arrived_sub_;
  bool navigation_arrived_ = false;
  void navigationArrivedCallback(const move_base_msgs::MoveBaseActionResult::ConstPtr& msg);

  // Helper functions
  void initializeDevice();
  void initializeROS();
  void readSerialData();
  void publishDoorStates();
  void cabinetContentCallback(const navigation_msgs::CabinetContentArray::ConstPtr& msg);
  bool hasStateChanged() const;
  bool hasContentsChanged() const;

public:
  // Public methods for controlling boxes and trajectories
  void sendDoorCommand(int boxIndex);
  bool sendTrajectoryRequest(const QString& path_name);
  const uint8_t* getDoorStates() const { return door_state; }
  const std::vector<navigation_msgs::CabinetContent>& getCurrentContents() const { return current_contents_; }
};

} // namespace gbx_rqt_interact
