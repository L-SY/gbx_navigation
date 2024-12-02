#include "gbx_delivery_rqt/main_window.h"
#include "ui_main_window.h"
#include <QPushButton>
#include <QMessageBox>
#include <QRegExpValidator>
#include <pluginlib/class_list_macros.h>

namespace gbx_delivery_rqt {

MainWindow::MainWindow()
    : rqt_gui_cpp::Plugin()
      , ui(new Ui::MainWindow)
      , widget_(nullptr)
      , returnTimer(new QTimer(this))
      , currentMode(NONE)
      , selectedBoxId(-1)
{
  setObjectName("DeliveryRobotPlugin");
}

MainWindow::~MainWindow()
{
  delete ui;
  if(widget_) {
    delete widget_;
  }
}

void MainWindow::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new QMainWindow();
  ui->setupUi(widget_);

  setupUi();
  setupBackground();
  setupConnections();
  setupBoxButtons();
  setupDestinationButtons();

  // 设置定时器
  returnTimer->setInterval(5000);  // 5秒
  returnTimer->setSingleShot(true);
  connect(returnTimer, &QTimer::timeout, this, &MainWindow::switchToMainPage);

  // 初始化页面
  currentPage = MAIN_PAGE;
  ui->stackedWidget->setCurrentWidget(ui->mainPage);

  context.addWidget(widget_);
}

void MainWindow::setupUi()
{
  // 为每个页面添加返回按钮
  QMap<QWidget*, QPair<QWidget*, QString>> pageNavigation;  // <当前页面, <上一页面, 页面标题>>

  // 设置页面导航关系
  pageNavigation[ui->deliveryPhonePage] = qMakePair(ui->mainPage, QString("寄件手机号输入"));
  pageNavigation[ui->pickupPhonePage] = qMakePair(ui->mainPage, QString("取件手机号输入"));
  pageNavigation[ui->boxSelectionPage] = qMakePair((QWidget*)nullptr, QString("选择快递柜"));
  pageNavigation[ui->destinationPage] = qMakePair(ui->boxSelectionPage, QString("选择目的地"));
  pageNavigation[ui->boxOpenPage] = qMakePair((QWidget*)nullptr, QString("箱门操作"));
  pageNavigation[ui->doorClosedPage] = qMakePair(ui->boxOpenPage, QString("完成"));

  // 为每个页面添加导航按钮
  for (auto it = pageNavigation.begin(); it != pageNavigation.end(); ++it) {
    QWidget* currentPage = it.key();
    QWidget* previousPage = it.value().first;

    // 创建水平布局来放置导航按钮
    QHBoxLayout* navLayout = new QHBoxLayout();

    // 创建返回主页按钮
    QPushButton* homeButton = new QPushButton(QString("返回主页"), widget_);
    homeButton->setMinimumSize(120, 50);
    homeButton->setFont(QFont("Arial", 14, QFont::Bold));
    homeButton->setStyleSheet("QPushButton { background-color: #FF9933; color: white; border-radius: 5px; }");
    connect(homeButton, &QPushButton::clicked, this, &MainWindow::switchToMainPage);
    navLayout->addWidget(homeButton);

    // 如果存在上一页且不是主页，添加返回上一页按钮
    if (previousPage != nullptr && previousPage != ui->mainPage) {
      QPushButton* backButton = new QPushButton(QString("返回上一步"), widget_);
      backButton->setMinimumSize(120, 50);
      backButton->setFont(QFont("Arial", 14, QFont::Bold));
      backButton->setStyleSheet("QPushButton { background-color: #4A90E2; color: white; border-radius: 5px; }");

      // 特殊处理箱子选择页面和箱门操作页面的返回逻辑
      if (currentPage == ui->boxSelectionPage || currentPage == ui->boxOpenPage) {
        connect(backButton, &QPushButton::clicked, this, [this]() {
          if (currentMode == DELIVERY) {
            ui->stackedWidget->setCurrentWidget(ui->deliveryPhonePage);
          } else {
            ui->stackedWidget->setCurrentWidget(ui->pickupPhonePage);
          }
        });
      } else {
        connect(backButton, &QPushButton::clicked, this, [this, previousPage]() {
          ui->stackedWidget->setCurrentWidget(previousPage);
        });
      }
      navLayout->addWidget(backButton);
    }

    // 添加一个弹簧来推动按钮到左侧
    navLayout->addStretch();

    // 获取页面的垂直布局
    QVBoxLayout* pageLayout = qobject_cast<QVBoxLayout*>(currentPage->layout());
    if (pageLayout) {
      // 在最上方插入导航按钮布局
      pageLayout->insertLayout(0, navLayout);
    }
  }

  // 创建数字键盘的按钮
  QVector<QPushButton*> numButtons;  // 存储按钮指针以便重复使用
  for (int i = 0; i < 12; i++) {
    QPushButton* button = new QPushButton(widget_);
    button->setMinimumSize(80, 80);
    button->setFont(QFont("Arial", 20, QFont::Bold));

    if (i < 9) {
      button->setText(QString::number(i + 1));
      button->setStyleSheet("QPushButton { background-color: #F0F0F0; border-radius: 5px; }");
    } else if (i == 9) {
      button->setText(QString("删除"));
      button->setStyleSheet("QPushButton { background-color: #FF6B6B; color: white; border-radius: 5px; }");
    } else if (i == 10) {
      button->setText(QString("0"));
      button->setStyleSheet("QPushButton { background-color: #F0F0F0; border-radius: 5px; }");
    } else {  // i == 11, 确认按钮
      button->setText(QString("确认"));
      button->setStyleSheet("QPushButton { background-color: #4CAF50; color: white; border-radius: 5px; }");
    }

    numButtons.append(button);

    // 连接按钮信号
    if (i == 9) { // 删除按钮
      connect(button, &QPushButton::clicked, this, [this]() {
        QLineEdit* currentEdit = nullptr;
        if (ui->stackedWidget->currentWidget() == ui->deliveryPhonePage) {
          currentEdit = ui->deliveryPhoneEdit;
        } else if (ui->stackedWidget->currentWidget() == ui->pickupPhonePage) {
          currentEdit = ui->pickupPhoneEdit;
        }

        if (currentEdit && !currentEdit->text().isEmpty()) {
          currentEdit->setText(currentEdit->text().left(currentEdit->text().length() - 1));
        }
      });
    } else if (i == 11) { // 确认按钮
      connect(button, &QPushButton::clicked, this, &MainWindow::handlePhoneNumberSubmit);
    } else { // 数字按钮
      connect(button, &QPushButton::clicked, this, [this, i]() {
        QString num = (i == 10) ? QString("0") : QString::number(i + 1);
        QLineEdit* currentEdit = nullptr;
        if (ui->stackedWidget->currentWidget() == ui->deliveryPhonePage) {
          currentEdit = ui->deliveryPhoneEdit;
        } else if (ui->stackedWidget->currentWidget() == ui->pickupPhonePage) {
          currentEdit = ui->pickupPhoneEdit;
        }

        if (currentEdit) {
          QString currentText = currentEdit->text();
          int maxLen = (currentEdit == ui->pickupPhoneEdit) ? 4 : 11;
          if (currentText.length() < maxLen) {
            currentEdit->setText(currentText + num);
          }
        }
      });
    }
  }

  // 将按钮添加到寄件页面的数字键盘布局
  for (int i = 0; i < numButtons.size(); i++) {
    int row = i / 3;
    int col = i % 3;
    ui->numpadLayout->addWidget(numButtons[i], row, col);
  }

  // 将按钮克隆并添加到取件页面的数字键盘布局
  for (int i = 0; i < numButtons.size(); i++) {
    QPushButton* cloneButton = new QPushButton(widget_);
    cloneButton->setMinimumSize(80, 80);
    cloneButton->setFont(QFont("Arial", 20, QFont::Bold));
    cloneButton->setText(numButtons[i]->text());
    cloneButton->setStyleSheet(numButtons[i]->styleSheet());

    // 复制信号连接
    if (i == 9) { // 删除按钮
      connect(cloneButton, &QPushButton::clicked, this, [this]() {
        QLineEdit* currentEdit = nullptr;
        if (ui->stackedWidget->currentWidget() == ui->deliveryPhonePage) {
          currentEdit = ui->deliveryPhoneEdit;
        } else if (ui->stackedWidget->currentWidget() == ui->pickupPhonePage) {
          currentEdit = ui->pickupPhoneEdit;
        }

        if (currentEdit && !currentEdit->text().isEmpty()) {
          currentEdit->setText(currentEdit->text().left(currentEdit->text().length() - 1));
        }
      });
    } else if (i == 11) { // 确认按钮
      connect(cloneButton, &QPushButton::clicked, this, &MainWindow::handlePhoneNumberSubmit);
    } else { // 数字按钮
      connect(cloneButton, &QPushButton::clicked, this, [this, i]() {
        QString num = (i == 10) ? QString("0") : QString::number(i + 1);
        QLineEdit* currentEdit = nullptr;
        if (ui->stackedWidget->currentWidget() == ui->deliveryPhonePage) {
          currentEdit = ui->deliveryPhoneEdit;
        } else if (ui->stackedWidget->currentWidget() == ui->pickupPhonePage) {
          currentEdit = ui->pickupPhoneEdit;
        }

        if (currentEdit) {
          QString currentText = currentEdit->text();
          int maxLen = (currentEdit == ui->pickupPhoneEdit) ? 4 : 11;
          if (currentText.length() < maxLen) {
            currentEdit->setText(currentText + num);
          }
        }
      });
    }

    int row = i / 3;
    int col = i % 3;
    ui->pickupNumpadLayout->addWidget(cloneButton, row, col);
  }

  // 设置输入验证器
  QRegExpValidator* fullPhoneValidator = new QRegExpValidator(QRegExp("^[0-9]{11}$"), this);
  QRegExpValidator* shortPhoneValidator = new QRegExpValidator(QRegExp("^[0-9]{4}$"), this);
  ui->deliveryPhoneEdit->setValidator(fullPhoneValidator);
  ui->pickupPhoneEdit->setValidator(shortPhoneValidator);
}

void MainWindow::setupBoxButtons()
{
  // 创建9个箱子按钮
  for (int i = 0; i < 9; i++) {
    QPushButton* button = new QPushButton(widget_);
    button->setText(QString("箱子 %1").arg(i + 1));
    button->setMinimumSize(120, 80);
    button->setFont(QFont("Arial", 16, QFont::Bold));

    connect(button, &QPushButton::clicked, this, [this, i]() {
      handleBoxSelection(i + 1);
    });

    int row = i / 3;
    int col = i % 3;
    ui->boxButtonsLayout->addWidget(button, row, col);
  }
}

void MainWindow::setupDestinationButtons()
{
  // 创建目的地按钮
  QStringList destinations = {"A区", "B区", "C区", "D区"};
  for (int i = 0; i < destinations.size(); i++) {
    QPushButton* button = new QPushButton(widget_);
    button->setText(destinations[i]);
    button->setMinimumSize(150, 80);
    button->setFont(QFont("Arial", 16, QFont::Bold));

    connect(button, &QPushButton::clicked, this, [this, i]() {
      handleDestinationSelect(i + 1);
    });

    int row = i / 2;
    int col = i % 2;
    ui->destinationButtonsLayout->addWidget(button, row, col);
  }
}

void MainWindow::setupConnections()
{
  // 基本页面切换
  connect(ui->pickupButton, &QPushButton::clicked, this, &MainWindow::switchToPickupMode);
  connect(ui->deliveryButton, &QPushButton::clicked, this, &MainWindow::switchToDeliveryMode);

  // 箱门状态变化
  connect(this, &MainWindow::boxOpened, this, &MainWindow::switchToBoxOpen);
  connect(this, &MainWindow::boxClosed, this, &MainWindow::switchToDoorClosed);
}

void MainWindow::switchToPickupMode()
{
  currentMode = PICKUP;
  currentPage = PICKUP_PHONE_PAGE;
  ui->stackedWidget->setCurrentWidget(ui->pickupPhonePage);
  ui->pickupPhoneEdit->clear();
}

void MainWindow::switchToDeliveryMode()
{
  currentMode = DELIVERY;
  currentPage = DELIVERY_PHONE_PAGE;
  ui->stackedWidget->setCurrentWidget(ui->deliveryPhonePage);
  ui->deliveryPhoneEdit->clear();
}

void MainWindow::switchToNextPage()
{
  switch (currentPage) {
  case DELIVERY_PHONE_PAGE:
    switchToBoxSelection();
    break;
  case BOX_SELECTION_PAGE:
    if (currentMode == DELIVERY) {
      switchToDestination();
    }
    break;
  case PICKUP_PHONE_PAGE:
    switchToBoxSelection();
    break;
  default:
    break;
  }
}

void MainWindow::switchToBoxSelection()
{
  currentPage = BOX_SELECTION_PAGE;
  ui->stackedWidget->setCurrentWidget(ui->boxSelectionPage);
}

void MainWindow::switchToDestination()
{
  currentPage = DESTINATION_PAGE;
  ui->stackedWidget->setCurrentWidget(ui->destinationPage);
}

void MainWindow::switchToBoxOpen()
{
  currentPage = BOX_OPEN_PAGE;
  ui->stackedWidget->setCurrentWidget(ui->boxOpenPage);
  ui->boxOpenLabel->setText(QString("箱子%1已打开\n取/放物品后请关门").arg(selectedBoxId));
}

void MainWindow::switchToDoorClosed()
{
  currentPage = DOOR_CLOSED_PAGE;
  ui->stackedWidget->setCurrentWidget(ui->doorClosedPage);
  returnTimer->start(); // 5秒后返回主页
}

void MainWindow::switchToMainPage()
{
  currentMode = NONE;
  currentPage = MAIN_PAGE;
  selectedBoxId = -1;
  ui->stackedWidget->setCurrentWidget(ui->mainPage);
  ui->deliveryPhoneEdit->clear();
  ui->pickupPhoneEdit->clear();
  returnTimer->stop();
}

void MainWindow::handlePhoneNumberSubmit()
{
  if (currentPage == DELIVERY_PHONE_PAGE) {
    QString phone = ui->deliveryPhoneEdit->text();
    if (validatePhoneNumber(phone)) {
      lastPhoneNumber = phone;  // 存储手机号
      // TODO: 验证手机号是否在服务器记录中
      switchToNextPage();
    } else {
      ui->deliveryPhoneEdit->clear();
      showErrorMessage("手机号输入有误，请检查");
    }
  } else if (currentPage == PICKUP_PHONE_PAGE) {
    QString shortPhone = ui->pickupPhoneEdit->text();
    if (validatePhoneNumber(shortPhone, true)) {
      // TODO: 验证手机号后四位是否匹配
      switchToNextPage();
    } else {
      ui->pickupPhoneEdit->clear();
      showErrorMessage("手机号输入有误，请检查");
    }
  }
}

void MainWindow::handleBoxSelection(int box)
{
  selectedBoxId = box;
  // TODO: 发送开门命令
  emit boxOpened(box);
}

void MainWindow::handleDestinationSelect(int destination)
{
  emit destinationSelected(destination);
  // TODO: 处理目的地选择，启动导航
}

void MainWindow::handleDoorStateChange(bool isOpen, int boxId)
{
  updateDoorStatus(boxId, isOpen);
  if (!isOpen) {
    emit boxClosed(boxId);
  }
}

void MainWindow::updateDoorStatus(int boxId, bool isOpen)
{
  // TODO: 更新箱门状态显示
}

bool MainWindow::validatePhoneNumber(const QString& phone, bool isShortPhone)
{
  if (isShortPhone) {
    return phone.length() == 4 && phone.toInt() != 0;
  } else {
    return phone.length() == 11 && phone.toInt() != 0;
  }
}

void MainWindow::showErrorMessage(const QString& message)
{
  QMessageBox::warning(widget_, "错误", message);
}

void MainWindow::handleTimeout()
{
  switchToMainPage();
}

void MainWindow::setupBackground()
{
  // TODO: 设置背景
}

void MainWindow::shutdownPlugin()
{
}

void MainWindow::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                              qt_gui_cpp::Settings& instance_settings) const
{
}

void MainWindow::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                 const qt_gui_cpp::Settings& instance_settings)
{
}

} // namespace gbx_delivery_rqt

PLUGINLIB_EXPORT_CLASS(gbx_delivery_rqt::MainWindow, rqt_gui_cpp::Plugin)