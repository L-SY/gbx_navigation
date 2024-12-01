#pragma once

#include <QObject>
#include <QSerialPort>
#include <QPushButton>
#include <QString>
#include <QTimer>
#include <QQueue>
#include <QMap>
#include <QVector>
#include <QSignalMapper>
#include <ros/ros.h>
#include <navigation_msgs/pub_trajectory.h>
#include <navigation_msgs/CabinetDoorArray.h>
#include <navigation_msgs/CabinetContentArray.h>

namespace Ui {
class MainWindow;
}

namespace gbx_rqt_interact {

class SerialNode : public QObject
{
  Q_OBJECT
public:
  explicit SerialNode(QObject *parent = nullptr);
  virtual ~SerialNode();

  void Init(Ui::MainWindow* _mainWindow_ui);

private slots:
  void handleButtonClick(int buttonIndex);
  void handleBoxButtonClick();
  void handleDestButtonClick();
  void UpdateUI();
  void processROSData();
  void onSerialDataReady();
  void onSerialError(QSerialPort::SerialPortError error);

signals:
  void requestUIUpdate();
  void serialErrorOccurred(const QString& errorMessage);

private:
  void DeviceInit();
  void UiInit();
  void ROSInit();
  void start();
  void readSerialData();
  void publishDoorStates();
  void cabinetContentCallback(const navigation_msgs::CabinetContentArray::ConstPtr& msg);
  void updateButton(int index);
  void sendBoxCommand(int boxIndex);
  void sendTrajectoryRequest(const QString& path_name);
  bool tryReconnectSerial();
  void logMessage(const QString& message, bool isError = false);

  Ui::MainWindow* mainWindow_ui;
  QSerialPort* serial;
  QStringList labels = {"A", "B", "C", "D", "E", "F"};
  QTimer* rosTimer;
  QSignalMapper* buttonMapper;

  bool show_box_flag;
  bool show_dest_flag;
  bool send_set_flag;
  bool button_update_flag;

  uint8_t box_fdb_state[8];
  uint8_t box_set_state[8];
  uint8_t last_box_state[8];

  QVector<bool> button_dirty_flags_;
  QMap<int, uint8_t> changed_states_;
  QQueue<QByteArray> serialDataBuffer;

  static const int SERIAL_BUFFER_SIZE = 1024;
  static const int SERIAL_RECONNECT_TIMEOUT = 5000; // 5 seconds
  static const QString DEFAULT_PORT_NAME;
  static const int DEFAULT_BAUD_RATE = 115200;

  ros::NodeHandle* nh_;
  ros::ServiceClient trajectory_client_;
  ros::Publisher door_state_pub_;
  ros::Subscriber cabinet_content_sub_;
  navigation_msgs::CabinetDoorArray door_states_msg_;

  std::vector<navigation_msgs::CabinetContent> current_contents_;
  std::vector<navigation_msgs::CabinetContent> last_contents_;

  QString current_serial_port_;
  int reconnect_attempts_;
  QTimer* reconnectTimer;
};

} // namespace gbx_rqt_interact