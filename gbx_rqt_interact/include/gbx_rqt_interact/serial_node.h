#pragma once

#include <QThread>
#include <QSerialPort>
#include <QObject>
#include <QPushButton>
#include <QString>

// 前向声明
namespace Ui {
class MainWindow;
}

namespace gbx_rqt_interact {

class SerialNode : public QThread
{
  Q_OBJECT
public:
  explicit SerialNode(QObject *parent = nullptr);
  ~SerialNode();

  void Init(Ui::MainWindow* _mainWindow_ui);
  void DeviceInit();
  void UiInit();

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

signals:
  void requestUIUpdate();

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
  void readSerialData();
};

} // namespace gbx_rqt_interact
