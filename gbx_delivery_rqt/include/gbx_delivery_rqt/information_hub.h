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
#include <navigation_msgs/CabinetContentArray.h>
#include <navigation_msgs/CabinetDoorArray.h>
#include <navigation_msgs/pub_trajectory.h>
#include <ros/ros.h>

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

signals:
  void doorStateChanged();
  void contentStateChanged();
  void trajectoryRequestResult(bool success, const QString& message);

private slots:
  void processSerialData();

protected:
  void run() override;

private:
  // Serial communication
  QSerialPort* serial;
  QTimer* updateTimer;
  uint8_t box_fdb_state[8];
  uint8_t box_set_state[8];
  uint8_t last_box_state[8];

  // ROS related
  ros::NodeHandle* nh_;
  ros::ServiceClient trajectory_client_;
  ros::Publisher door_state_pub_;
  ros::Subscriber cabinet_content_sub_;
  std::vector<navigation_msgs::CabinetContent> current_contents_;
  std::vector<navigation_msgs::CabinetContent> last_contents_;

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
  void sendBoxCommand(int boxIndex);
  void sendTrajectoryRequest(const QString& path_name);
  const uint8_t* getDoorStates() const { return box_fdb_state; }
  const std::vector<navigation_msgs::CabinetContent>& getCurrentContents() const { return current_contents_; }
};

} // namespace gbx_rqt_interact
