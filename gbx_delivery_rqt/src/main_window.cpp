#include "gbx_delivery_rqt/main_window.h"
#include "ui_main_window.h"
#include <QGridLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QRegExpValidator>
#include <pluginlib/class_list_macros.h>

namespace gbx_delivery_rqt {

MainWindow::MainWindow()
    : rqt_gui_cpp::Plugin()
      , ui(new Ui::MainWindow)
      , widget_(nullptr)
      , returnTimer(new QTimer(this))
      , currentMode(NONE)
      , selectedCabinetId(-1)
      , infoHub(nullptr)
{
  ROS_INFO_STREAM("MainWindow: Constructor starting...");
  try {
    setObjectName("DeliveryRobotPlugin");
    ROS_INFO_STREAM("MainWindow: Constructor completed successfully");
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("MainWindow: Exception in constructor: " << e.what());
    throw;
  } catch (...) {
    ROS_ERROR_STREAM("MainWindow: Unknown exception in constructor");
    throw;
  }
}

MainWindow::~MainWindow()
{
  delete ui;
  if(widget_) {
    delete widget_;
  }
}

void MainWindow::setupInfoHub()
{
  infoHub = new gbx_rqt_interact::InformationHub(this);

  // Connect signals from InformationHub
  connect(infoHub, &gbx_rqt_interact::InformationHub::doorStateChanged,
          this, &MainWindow::handleDoorStateUpdate);
  connect(infoHub, &gbx_rqt_interact::InformationHub::contentStateChanged,
          this, &MainWindow::handleContentsUpdate);
  connect(infoHub, &gbx_rqt_interact::InformationHub::trajectoryRequestResult,
          this, &MainWindow::handleTrajectoryResult);
  connect(infoHub, &gbx_rqt_interact::InformationHub::navigationArrived,
          this, &MainWindow::handleNavigationArrival);

  infoHub->init();
  infoHub->start();  // Start the QThread

  // Try to open serial port
  if (!infoHub->openSerialPort()) {
    showErrorMessage("无法打开串口，部分功能可能不可用");
  }
}

void MainWindow::initPlugin(qt_gui_cpp::PluginContext& context)
{
  ROS_INFO_STREAM("MainWindow: initPlugin starting...");

  try {
    // 创建主窗口
    ROS_INFO_STREAM("MainWindow: Creating main window");
    widget_ = new QMainWindow();
    if (!widget_) {
      ROS_ERROR_STREAM("Failed to create main window");
      throw std::runtime_error("Failed to create main window");
    }

    ROS_INFO_STREAM("MainWindow: Setting up UI");
    ui->setupUi(widget_);

    // 检查关键组件
    if (!ui || !ui->stackedWidget) {
      ROS_ERROR_STREAM("UI components not initialized properly");
      throw std::runtime_error("Failed to initialize stacked widget");
    }

    // 按顺序初始化各个组件
    ROS_INFO_STREAM("MainWindow: Setting up InfoHub");
    setupInfoHub();

    ROS_INFO_STREAM("MainWindow: Setting up basic UI components");
    setupUi();

    ROS_INFO_STREAM("MainWindow: Setting up background");
    setupBackground();

    ROS_INFO_STREAM("MainWindow: Setting up connections");
    setupConnections();

    ROS_INFO_STREAM("MainWindow: Setting up box buttons");
    setupCabinetButtons();

    ROS_INFO_STREAM("MainWindow: Setting up QR code page");
    setupQRCodePage();

    // 设置定时器
    ROS_INFO_STREAM("MainWindow: Setting up return timer");
    returnTimer->setInterval(5000);  // 5秒
    returnTimer->setSingleShot(true);
    connect(returnTimer, &QTimer::timeout, this, &MainWindow::switchToMainPage);

    // 初始化页面
    ROS_INFO_STREAM("MainWindow: Initializing main page");
    currentPage = MAIN_PAGE;
    ui->stackedWidget->setCurrentWidget(ui->mainPage);

    ROS_INFO_STREAM("MainWindow: Adding widget to context");
    context.addWidget(widget_);

    ROS_INFO_STREAM("MainWindow: initPlugin completed successfully");
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("MainWindow: Exception in initPlugin: " << e.what());
    throw;
  } catch (...) {
    ROS_ERROR_STREAM("MainWindow: Unknown exception in initPlugin");
    throw;
  }
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
  pageNavigation[ui->arrivalPage] = qMakePair(ui->destinationPage, QString("到达"));

  // 创建二维码页面
  QWidget* qrCodePage = new QWidget();
  qrCodePage->setObjectName("qrCodePage");
  QVBoxLayout* qrLayout = new QVBoxLayout(qrCodePage);
  ui->stackedWidget->addWidget(qrCodePage);
  pageNavigation[qrCodePage] = qMakePair(ui->doorClosedPage, QString("扫码取件"));

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
          } else if (currentMode == PICKUP) {
            ui->stackedWidget->setCurrentWidget(ui->pickupPhonePage);
          } else {
            ui->stackedWidget->setCurrentWidget(ui->mainPage);
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

  // 修改主页面上的取件和寄件按钮样式为绿色
  if (ui->pickupButton) {
    ui->pickupButton->setStyleSheet(
        "QPushButton {"
        "    background-color: #4CAF50;"  // 绿色
        "    color: white;"
        "    border-radius: 10px;"
        "}"
        "QPushButton:hover {"
        "    background-color: #45a049;"
        "}"
    );
  }

  if (ui->deliveryButton) {
    ui->deliveryButton->setStyleSheet(
        "QPushButton {"
        "    background-color: #4CAF50;"  // 绿色
        "    color: white;"
        "    border-radius: 10px;"
        "}"
        "QPushButton:hover {"
        "    background-color: #45a049;"
        "}"
    );
  }

  // 添加还箱按钮到主页
  QPushButton* returnBoxButton = new QPushButton("还箱", widget_);
  returnBoxButton->setMinimumSize(200, 100);
  returnBoxButton->setFont(QFont("Arial", 20, QFont::Bold));
  returnBoxButton->setStyleSheet(
      "QPushButton { background-color: #9370DB; color: white; border-radius: 10px; }"
      "QPushButton:hover { background-color: #8A2BE2; }"
  );
  connect(returnBoxButton, &QPushButton::clicked, this, &MainWindow::switchToReturnBoxMode);

  // 将还箱按钮添加到主页布局中
  QVBoxLayout* mainLayout = qobject_cast<QVBoxLayout*>(ui->mainPage->layout());
  if (mainLayout) {
    // 假设已有取件和寄件按钮，找到它们所在的布局
    for (int i = 0; i < mainLayout->count(); ++i) {
      QHBoxLayout* buttonLayout = qobject_cast<QHBoxLayout*>(mainLayout->itemAt(i)->layout());
      if (buttonLayout && buttonLayout->count() >= 2) {
        // 找到了取件和寄件按钮所在的布局，添加还箱按钮
        buttonLayout->addWidget(returnBoxButton);
        break;
      }
    }
  }
}

void MainWindow::setupCabinetButtons()
{
  QGridLayout* leftLayout = new QGridLayout();
  QGridLayout* rightLayout = new QGridLayout();
  QHBoxLayout* mainLayout = new QHBoxLayout();

  // 左侧 2,4,6
  for (int i = 0; i < 3; i++) {
    QPushButton* button = new QPushButton(widget_);
    button->setText(QString("箱子 %1").arg((i + 1) * 2));
    button->setMinimumSize(120, 80);
    button->setFont(QFont("Arial", 16, QFont::Bold));
    connect(button, &QPushButton::clicked, this, [this, i]() {
      handleBoxSelection((i + 1) * 2);
    });
    leftLayout->addWidget(button, i, 0);
    cabinetButtons.insert((i + 1) * 2, button);  // Store button reference
  }

  // 右侧 1,3,5
  for (int i = 0; i < 3; i++) {
    QPushButton* button = new QPushButton(widget_);
    button->setText(QString("箱子 %1").arg(i * 2 + 1));
    button->setMinimumSize(120, 80);
    button->setFont(QFont("Arial", 16, QFont::Bold));
    connect(button, &QPushButton::clicked, this, [this, i]() {
      handleBoxSelection(i * 2 + 1);
    });
    rightLayout->addWidget(button, i, 0);
    cabinetButtons.insert(i * 2 + 1, button);  // Store button reference
  }

  mainLayout->addLayout(leftLayout);
  mainLayout->addLayout(rightLayout);
  ui->boxButtonsLayout->addLayout(mainLayout, 0, 0);
}

void MainWindow::setupQRCodePage()
{
  ROS_INFO_STREAM("Setting up QR code page");

  // 完全删除旧页面并创建新页面
  QWidget* oldPage = ui->stackedWidget->findChild<QWidget*>("qrCodePage");
  if (oldPage) {
    ROS_INFO_STREAM("Removing old QR code page");
    ui->stackedWidget->removeWidget(oldPage);
    delete oldPage;
  }

  // 创建全新的页面
  QWidget* qrCodePage = new QWidget();
  qrCodePage->setObjectName("qrCodePage");

  // 创建布局
  QVBoxLayout* layout = new QVBoxLayout(qrCodePage);

  // 添加标题
  QLabel* titleLabel = new QLabel("取件后请还箱，添加客户微信了解返还押金5元", qrCodePage);
  titleLabel->setFont(QFont("Arial", 20, QFont::Bold));
  titleLabel->setAlignment(Qt::AlignCenter);
  layout->addWidget(titleLabel);

  // 创建二维码标签（不存储为成员变量）
  QLabel* qrLabel = new QLabel(qrCodePage);
  qrLabel->setObjectName("qrCodeImageLabel");
  qrLabel->setAlignment(Qt::AlignCenter);
  qrLabel->setMinimumSize(300, 300);

  // 加载二维码图片
  QPixmap qrPixmap("resources/images/qrcode.jpg");
  if (qrPixmap.isNull()) {
    ROS_WARN_STREAM("Failed to load QR code from resources/images/qrcode.jpg, trying alternative path");
    qrPixmap.load(":/images/qrcode.jpg");
  }

  if (!qrPixmap.isNull()) {
    ROS_INFO_STREAM("QR code image loaded successfully");
    qrLabel->setPixmap(qrPixmap.scaled(300, 300, Qt::KeepAspectRatio, Qt::SmoothTransformation));
  } else {
    ROS_ERROR_STREAM("Failed to load QR code image from any path");
    qrLabel->setText("二维码图片加载失败");
    qrLabel->setStyleSheet("QLabel { color: red; font-size: 18px; }");
  }
  layout->addWidget(qrLabel);

  // 添加返回按钮
  QPushButton* backButton = new QPushButton("返回主页", qrCodePage);
  backButton->setMinimumSize(150, 60);
  backButton->setFont(QFont("Arial", 16, QFont::Bold));
  backButton->setStyleSheet("QPushButton { background-color: #4A90E2; color: white; border-radius: 8px; }");
  connect(backButton, &QPushButton::clicked, this, &MainWindow::switchToMainPage);
  layout->addWidget(backButton, 0, Qt::AlignCenter);

  // 设置布局的间距和边距
  layout->setSpacing(20);
  layout->setContentsMargins(30, 30, 30, 30);

  // 添加到 stackedWidget
  ui->stackedWidget->addWidget(qrCodePage);
}

bool MainWindow::isBoxForDeliveryOrPickup(int boxId)
{
  // 只有5、6号柜子用于寄件和取件
  return boxId == 5 || boxId == 6;
}

void MainWindow::handleBoxSelection(int box)
{
  selectedCabinetId = box;

  // 检查当前模式和箱子是否匹配
  if ((currentMode == DELIVERY || currentMode == PICKUP) && !isBoxForDeliveryOrPickup(box)) {
    showErrorMessage("寄件和取件只能使用5、6号柜子，请重新选择");
    return;
  }

  if (infoHub) {
    if (currentMode == DELIVERY) {
      // 存储手机号后四位与柜子的对应关系
      QString lastFourDigits = lastPhoneNumber.right(4);
      phoneNumberToCabinet[lastFourDigits] = box;
    }

    infoHub->sendDoorCommand(box - 1);
    ui->boxOpenLabel->setText(QString("箱子%1已打开\n请%2物品").arg(box)
                                  .arg(currentMode == DELIVERY ? "放入" :
                                                               (currentMode == PICKUP ? "取出" : "归还")));
    switchToBoxOpen();
  }
}

void MainWindow::startWaitForObjectDetection()
{
  // 在寄件模式下，检测物品后直接进入目的地选择页面
  if (currentMode == DELIVERY) {
    QTimer::singleShot(1000, this, &MainWindow::switchToDestination);
  } else {
    // 其他模式可能的处理
    QTimer::singleShot(1000, this, &MainWindow::switchToDoorClosed);
  }
}

void MainWindow::handleDoorStateUpdate()
{
  if (!infoHub) return;

  const uint8_t* states = infoHub->getDoorStates();
  for (int i = 0; i < 6; i++) {
    updateDoorStatus(i + 1, states[i + 1] == 1);
  }

  // 如果是取件模式且检测到关门，从记录中移除对应关系
  if (currentMode == PICKUP && selectedCabinetId > 0 && states[selectedCabinetId] == 0) {
    QString lastFourDigits = ui->pickupPhoneEdit->text();
    phoneNumberToCabinet.remove(lastFourDigits);

    // 取件成功后，切换到二维码页面
    switchToQRCodePage();

    selectedCabinetId = -1;
  }
  // 如果是还箱模式下关门，切换到关门感谢页面
  else if (currentMode == RETURN_BOX && selectedCabinetId > 0 && states[selectedCabinetId] == 0) {
    switchToDoorClosed();
    selectedCabinetId = -1;
  }
  // 如果是寄件模式下关门，切换到目的地选择页面
  else if (currentMode == DELIVERY && selectedCabinetId > 0 && states[selectedCabinetId] == 0) {
    // 不要调用 switchToDoorClosed()，而是直接进入目的地选择页面
    switchToDestination();
  }
}

void MainWindow::handleContentsUpdate()
{
  if (!infoHub) return;

  const auto& contents = infoHub->getCurrentContents();
  updateCabinetAvailability(contents);
}

void MainWindow::updateCabinetAvailability(const std::vector<navigation_msgs::CabinetContent>& contents)
{
  // 首先将所有箱子设置为空闲状态
  for (auto it = cabinetButtons.begin(); it != cabinetButtons.end(); ++it) {
    bool isForDeliveryOrPickup = isBoxForDeliveryOrPickup(it.key());
    updateCabinetButtonStyle(it.value(), true, isForDeliveryOrPickup);
  }

  // 更新已占用的箱子状态
  for (const auto& content : contents) {
    try {
      // 检查 box_id 是否为空
      if (content.box.ascii_epc.empty()) {
        continue;  // 跳过空的 box_id
      }

      // 尝试解析 box_id
      int boxId = 0;
      try {
        boxId = std::stoi(content.box.ascii_epc);
      } catch (const std::exception& e) {
        ROS_ERROR_STREAM("Error parsing box_id: " << content.box.ascii_epc << " Error: " << e.what());
        continue;  // 跳过无效的 box_id
      }

      // 验证 boxId 的范围
      if (boxId <= 0 || boxId > cabinetButtons.size()) {
        ROS_ERROR_STREAM("Invalid box_id range: " << boxId);
        continue;
      }

      // 确保按钮存在
      if (cabinetButtons.contains(boxId)) {
        auto* button = cabinetButtons[boxId];
        if (button) {
          bool isForDeliveryOrPickup = isBoxForDeliveryOrPickup(boxId);
          updateCabinetButtonStyle(button, false, isForDeliveryOrPickup);
        }
      }
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Unexpected error in updateCabinetAvailability: " << e.what());
      continue;  // 继续处理下一个内容
    }
  }

  // 根据当前模式禁用不适用的箱子
  if (currentMode == DELIVERY || currentMode == PICKUP) {
    // 寄件和取件模式下，只启用5、6号柜子
    for (auto it = cabinetButtons.begin(); it != cabinetButtons.end(); ++it) {
      if (!isBoxForDeliveryOrPickup(it.key())) {
        it.value()->setEnabled(false);
        it.value()->setStyleSheet(
            "QPushButton {"
            "    background-color: #CCCCCC;"  // 灰色表示不可用
            "    color: #666666;"
            "    border-radius: 5px;"
            "}"
        );
      }
    }
  } else if (currentMode == RETURN_BOX) {
    // 还箱模式下，禁用5、6号柜子
    for (auto it = cabinetButtons.begin(); it != cabinetButtons.end(); ++it) {
      if (isBoxForDeliveryOrPickup(it.key())) {
        it.value()->setEnabled(false);
        it.value()->setStyleSheet(
            "QPushButton {"
            "    background-color: #CCCCCC;"  // 灰色表示不可用
            "    color: #666666;"
            "    border-radius: 5px;"
            "}"
        );
      }
    }
  }
}

// 更新按钮样式
void MainWindow::updateCabinetButtonStyle(QPushButton* button, bool isEmpty, bool isForDeliveryOrPickup)
{
  if (!button) return;  // 添加空指针检查

  if (isEmpty) {
    button->setEnabled(true);
    if (isForDeliveryOrPickup) {
    // 寄件和取件专用箱子用蓝色标识
      button->setStyleSheet(
          "QPushButton {"
          "    background-color: #4A90E2;"  // 蓝色表示寄件取件专用
          "    color: white;"
          "    border-radius: 5px;"
          "}"
        "QPushButton:hover {"
        "    background-color: #3A80D2;"
        "}"
    );
    } else {
      // 其他箱子用绿色标识
      button->setStyleSheet(
          "QPushButton {"
          "    background-color: #4CAF50;"  // 绿色表示空箱
          "    color: white;"
          "    border-radius: 5px;"
          "}"
          "QPushButton:hover {"
          "    background-color: #45a049;"
          "}"
      );
    }
  } else {
    button->setEnabled(false);
    button->setStyleSheet(
        "QPushButton {"
        "    background-color: #f44336;"  // 红色表示有物品
        "    color: white;"
        "    border-radius: 5px;"
        "}"
    );
  }
}

void MainWindow::handleTrajectoryResult(bool success, const QString& message)
{
  if (!success) {
    showErrorMessage("导航请求失败: " + message);
  }
}

void MainWindow::updateDoorStatus(int boxId, bool isOpen)
{
  if (isOpen) {
    if (boxId == selectedCabinetId) {
      emit boxOpened(boxId);
    }
  } else {
    if (boxId == selectedCabinetId) {
      emit boxClosed(boxId);
    }
  }
}

void MainWindow::setupConnections()
{
  // 基本页面切换
  connect(ui->pickupButton, &QPushButton::clicked, this, &MainWindow::switchToPickupMode);
  connect(ui->deliveryButton, &QPushButton::clicked, this, &MainWindow::switchToDeliveryMode);
  connect(infoHub, &gbx_rqt_interact::InformationHub::navigationArrived,
          this, &MainWindow::handleNavigationArrival);

  // 箱门状态变化
  connect(this, &MainWindow::boxOpened, this, [this](int boxId) {
    currentPage = BOX_OPEN_PAGE;
    ui->stackedWidget->setCurrentWidget(ui->boxOpenPage);
    ui->boxOpenLabel->setText(QString("箱子%1已打开\n%2物品后请关门").arg(boxId)
                                  .arg(currentMode == DELIVERY ? "放入" :
                                                               (currentMode == PICKUP ? "取出" : "归还")));
    if (currentMode == PICKUP)
    {
      auto cabinetInfo = infoHub->getCabinetInfo();
      if (selectedCabinetId > 0 && selectedCabinetId <= static_cast<int>(cabinetInfo.size())) {
        infoHub->publishOutputDelivery(
            lastPhoneNumber.toStdString(),
            cabinetInfo[selectedCabinetId-1].box.ascii_epc,
            cabinetInfo[selectedCabinetId-1].box.ascii_epc,
            lastPhoneNumber.toStdString()
        );
      } else {
        ROS_ERROR_STREAM("Invalid cabinet ID: " << selectedCabinetId);
      }
    }
  });

  connect(this, &MainWindow::boxClosed, this, [this](int boxId) {
    if (currentMode == DELIVERY && currentPage == BOX_OPEN_PAGE) {
      ui->boxOpenLabel->setText(QString("箱子%1已关闭\n正在检测物品...").arg(boxId));
      startWaitForObjectDetection();
    } else if (currentMode == PICKUP && currentPage == BOX_OPEN_PAGE) {
      // 取件模式下关门后切换到二维码页面
      switchToQRCodePage();
    } else {
      switchToDoorClosed();
    }
  });
}

void MainWindow::setupDestinationPage()
{
  // 获取现有的布局
  QVBoxLayout* mainLayout = qobject_cast<QVBoxLayout*>(ui->destinationPage->layout());
  if (!mainLayout) {
    mainLayout = new QVBoxLayout(ui->destinationPage);
  } else {
    // 保存第一个布局项（导航按钮的布局）
    QLayoutItem* navItem = mainLayout->takeAt(0);

    // 清除其他布局项
    QLayoutItem* item;
    while ((item = mainLayout->takeAt(0)) != nullptr) {
      delete item->widget();
      delete item;
    }

    // 修改导航按钮布局的间距
    if (navItem) {
      QHBoxLayout* navLayout = qobject_cast<QHBoxLayout*>(navItem->layout());
      if (navLayout) {
        navLayout->setSpacing(10);  // 设置导航按钮之间的间距
        navLayout->setContentsMargins(10, 10, 10, 20);  // 设置导航按钮区域的边距
      }
      mainLayout->addItem(navItem);
    }
  }

  mainLayout->setSpacing(10);  // 设置组件之间的间距
  mainLayout->setContentsMargins(30, 20, 30, 20);  // 设置页面边距

  // 创建标题标签
  QLabel* titleLabel = new QLabel("请选择送货目的地", ui->destinationPage);
  titleLabel->setFont(QFont("Microsoft YaHei", 24, QFont::Bold));
  titleLabel->setAlignment(Qt::AlignCenter);
  mainLayout->addWidget(titleLabel);

  // 创建地图容器
  QWidget* mapContainer = new QWidget(ui->destinationPage);
  mapContainer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  QVBoxLayout* mapLayout = new QVBoxLayout(mapContainer);
  mapLayout->setSpacing(5);  // 减小地图内部的间距

  // 设置地图
  QLabel* mapLabel = new QLabel(mapContainer);
  QPixmap mapPixmap(":/images/gbx_rviz.png");
  if (!mapPixmap.isNull()) {
    mapLabel->setPixmap(mapPixmap.scaled(800, 600, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    mapLabel->setAlignment(Qt::AlignCenter);
  } else {
    ROS_ERROR_STREAM("Failed to load map image from path: :/images/2F_whole.png");
    mapLabel->setText("地图加载失败");
    mapLabel->setStyleSheet("QLabel { background-color: #f0f0f0; padding: 20px; }");
  }
  mapLayout->addWidget(mapLabel, 0, Qt::AlignCenter);

  // 添加说明文字到地图容器下方
  QLabel* hintLabel = new QLabel("点击对应区域按钮选择送货目的地", mapContainer);
  hintLabel->setFont(QFont("Microsoft YaHei", 12));
  hintLabel->setAlignment(Qt::AlignCenter);
  mapLayout->addWidget(hintLabel);

  mainLayout->addWidget(mapContainer);

  // 创建按钮容器
  QWidget* buttonContainer = new QWidget(ui->destinationPage);
  QVBoxLayout* buttonContainerLayout = new QVBoxLayout(buttonContainer);

  // 创建区域按钮的容器
  QWidget* areaButtonsWidget = new QWidget(buttonContainer);
  QHBoxLayout* buttonLayout = new QHBoxLayout(areaButtonsWidget);
  buttonLayout->setSpacing(15);  // 按钮之间的间距

  // 创建区域按钮
  QStringList areas = {"A区", "B区", "C区", "D区", "E区", "F区", "G区"};
  selectedDestination = -1; // 重置选择的目的地

  for (int i = 0; i < areas.size(); ++i) {
    const QString& area = areas[i];
    QPushButton* button = new QPushButton(area, areaButtonsWidget);
    button->setMinimumSize(100, 60);
    button->setFont(QFont("Microsoft YaHei", 14, QFont::Bold));
    button->setStyleSheet(
        "QPushButton {"
        "    background-color: #4A90E2;"
        "    color: white;"
        "    border-radius: 8px;"
        "    padding: 10px;"
        "}"
        "QPushButton:hover {"
        "    background-color: #357ABD;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #2A5A8E;"
        "}"
    );

    // 使用索引直接连接槽函数，但只是选择目的地，不立即发送
    connect(button, &QPushButton::clicked, this, [this, i, button, areas]() {
      // 重置所有按钮样式
      for (auto it = destinationButtons.begin(); it != destinationButtons.end(); ++it) {
        it.value()->setStyleSheet(
            "QPushButton {"
            "    background-color: #4A90E2;"
            "    color: white;"
            "    border-radius: 8px;"
            "    padding: 10px;"
            "}"
            "QPushButton:hover {"
            "    background-color: #357ABD;"
            "}"
            "QPushButton:pressed {"
            "    background-color: #2A5A8E;"
            "}"
        );
      }

      // 设置选中按钮的样式
      button->setStyleSheet(
          "QPushButton {"
          "    background-color: #FF9933;" // 橙色表示选中
          "    color: white;"
          "    border-radius: 8px;"
          "    padding: 10px;"
          "}"
      );

      selectedDestination = i;
      destinationConfirmButton->setEnabled(true);
      destinationInfoLabel->setText(QString("已选择: %1").arg(areas[i]));
    });

    buttonLayout->addWidget(button);
    destinationButtons[i] = button; // 存储按钮引用
  }

  buttonContainerLayout->addWidget(areaButtonsWidget);

  // 添加信息标签
  destinationInfoLabel = new QLabel("请选择一个目的地", buttonContainer);
  destinationInfoLabel->setFont(QFont("Microsoft YaHei", 14));
  destinationInfoLabel->setAlignment(Qt::AlignCenter);
  buttonContainerLayout->addWidget(destinationInfoLabel);

  // 添加确认和取消按钮
  QWidget* confirmCancelWidget = new QWidget(buttonContainer);
  QHBoxLayout* confirmCancelLayout = new QHBoxLayout(confirmCancelWidget);

  // 取消按钮
  QPushButton* cancelButton = new QPushButton("取消", confirmCancelWidget);
  cancelButton->setMinimumSize(120, 50);
  cancelButton->setFont(QFont("Microsoft YaHei", 14, QFont::Bold));
  cancelButton->setStyleSheet(
      "QPushButton {"
      "    background-color: #FF6B6B;"
      "    color: white;"
      "    border-radius: 8px;"
      "    padding: 10px;"
      "}"
      "QPushButton:hover {"
      "    background-color: #E55555;"
      "}"
  );
  connect(cancelButton, &QPushButton::clicked, this, [this]() {
    // 重置选择
    selectedDestination = -1;
    for (auto it = destinationButtons.begin(); it != destinationButtons.end(); ++it) {
      it.value()->setStyleSheet(
          "QPushButton {"
          "    background-color: #4A90E2;"
          "    color: white;"
          "    border-radius: 8px;"
          "    padding: 10px;"
          "}"
          "QPushButton:hover {"
          "    background-color: #357ABD;"
          "}"
          "QPushButton:pressed {"
          "    background-color: #2A5A8E;"
          "}"
      );
    }
    destinationInfoLabel->setText("请选择一个目的地");
    destinationConfirmButton->setEnabled(false);
  });
  confirmCancelLayout->addWidget(cancelButton);

  // 确认按钮
  destinationConfirmButton = new QPushButton("确认", confirmCancelWidget);
  destinationConfirmButton->setMinimumSize(120, 50);
  destinationConfirmButton->setFont(QFont("Microsoft YaHei", 14, QFont::Bold));
  destinationConfirmButton->setStyleSheet(
      "QPushButton {"
      "    background-color: #4CAF50;"
      "    color: white;"
      "    border-radius: 8px;"
      "    padding: 10px;"
      "}"
      "QPushButton:hover {"
      "    background-color: #45a049;"
      "}"
      "QPushButton:disabled {"
      "    background-color: #CCCCCC;"
      "    color: #666666;"
      "}"
  );
  destinationConfirmButton->setEnabled(false);
  connect(destinationConfirmButton, &QPushButton::clicked, this, [this]() {
    if (selectedDestination >= 0) {
      confirmDestinationSelection(selectedDestination);
    }
  });
  confirmCancelLayout->addWidget(destinationConfirmButton);

  buttonContainerLayout->addWidget(confirmCancelWidget);

  mainLayout->addWidget(buttonContainer);

  // 设置布局的拉伸因子
  mainLayout->setStretchFactor(mapContainer, 3);
  mainLayout->setStretchFactor(buttonContainer, 1);

  // 更新界面
  ui->destinationPage->update();
}

void MainWindow::confirmDestinationSelection(int destination)
{
  emit destinationSelected(destination);
  if (infoHub) {
    // 使用新的目标点发送方法
    if (infoHub->sendTargetPoint(destination)) {
      auto cabinerInfo = infoHub->getCabinetInfo();
      if (selectedCabinetId > 0 && selectedCabinetId <= static_cast<int>(cabinerInfo.size())) {
        infoHub->publishIndoorDeliveryOrder(
            "IndoorCar",
            cabinerInfo[selectedCabinetId-1].box.ascii_epc,
            cabinerInfo[selectedCabinetId-1].box.ascii_epc,
            "IndoorCar",
            QString(QChar('A' + destination)).toStdString(),
            "No.1",  // 机器人ID作为owner
            lastPhoneNumber.toStdString(), // 接收者电话
            "",                         // 接收者姓名（可选）
            ""                          // 发送者姓名（可选）
        );
      } else {
        ROS_ERROR_STREAM("Invalid cabinet ID for destination select: " << selectedCabinetId);
      }
    }
  }

  QMessageBox::information(widget_, "提示", "目标点已发送,机器人即将启动");
  QTimer::singleShot(2000, this, &MainWindow::switchToMainPage);
}


void MainWindow::handleNavigationArrival(bool arrived) {
  if (!arrived) return;

  if (infoHub) {
    auto cabinetInfo = infoHub->getCabinetInfo();
    if (selectedCabinetId > 0 && selectedCabinetId <= static_cast<int>(cabinetInfo.size())) {
      infoHub->publishOutputDelivery(
          "IndoorCar",
          cabinetInfo[selectedCabinetId-1].box.ascii_epc,
          cabinetInfo[selectedCabinetId-1].box.ascii_epc,
          lastPhoneNumber.toStdString()
      );
    } else {
      ROS_ERROR_STREAM("Invalid cabinet ID for navigation arrival: " << selectedCabinetId);
    }
  }

  currentPage = ARRIVAL_PAGE;
  // 切换到到达页面
  ui->stackedWidget->setCurrentWidget(ui->arrivalPage);
  returnTimer->start(); // 5秒后返回主页
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
  // 处理标记的点击事件
  for (int i = 0; i < destinationMarkers.size(); i++) {
    if (obj == destinationMarkers[i]) {
      if (event->type() == QEvent::MouseButtonPress) {
        handleDestinationSelect(i);  // 使用 i 作为目的地索引
        return true;
      }
    }
  }

  return QObject::eventFilter(obj, event);
}

void MainWindow::switchToDestination()
{
  currentPage = DESTINATION_PAGE;
  ui->stackedWidget->setCurrentWidget(ui->destinationPage);
  setupDestinationPage();
}

void MainWindow::switchToQRCodePage()
{
  ROS_INFO_STREAM("Switching to QR code page");
  try {
    currentPage = QR_CODE_PAGE;

    // 每次切换都重新创建二维码页面
    setupQRCodePage();

    // 查找新创建的页面
    QWidget* qrCodePage = ui->stackedWidget->findChild<QWidget*>("qrCodePage");
    if (qrCodePage) {
      ui->stackedWidget->setCurrentWidget(qrCodePage);
      returnTimer->start(); // 5秒后返回主页
      ROS_INFO_STREAM("Successfully switched to QR code page");
    } else {
      ROS_ERROR_STREAM("QR code page not found after setup");
      switchToDoorClosed(); // 如果找不到二维码页面，就切换到关门感谢页面
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception in switchToQRCodePage: " << e.what());
    switchToDoorClosed(); // 出现异常时切换到关门感谢页面
  } catch (...) {
    ROS_ERROR_STREAM("Unknown exception in switchToQRCodePage");
    switchToDoorClosed(); // 出现异常时切换到关门感谢页面
  }
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

void MainWindow::switchToReturnBoxMode()
{
  currentMode = RETURN_BOX;
  currentPage = BOX_SELECTION_PAGE;
  ui->stackedWidget->setCurrentWidget(ui->boxSelectionPage);
  // 更新箱子状态，禁用5、6号柜子
  handleContentsUpdate();
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
  // 更新箱子状态，根据当前模式启用/禁用相应的箱子
  handleContentsUpdate();
}

void MainWindow::switchToBoxOpen()
{
  currentPage = BOX_OPEN_PAGE;
  ui->stackedWidget->setCurrentWidget(ui->boxOpenPage);
  ui->boxOpenLabel->setText(QString("箱子%1已打开\n%2物品后请关门").arg(selectedCabinetId)
                                .arg(currentMode == DELIVERY ? "放入" :
                                                             (currentMode == PICKUP ? "取出" : "归还")));
}

void MainWindow::switchToDoorClosed()
{
  // 如果是寄件模式，不应该调用此函数
  if (currentMode == DELIVERY) {
    ROS_WARN_STREAM("Attempted to switch to door closed page in delivery mode, switching to destination page instead");
    switchToDestination();
    return;
  }

  currentPage = DOOR_CLOSED_PAGE;
  ui->stackedWidget->setCurrentWidget(ui->doorClosedPage);
  returnTimer->start(); // 5秒后返回主页
}

void MainWindow::switchToMainPage()
{
  currentMode = NONE;
  currentPage = MAIN_PAGE;
  ui->stackedWidget->setCurrentWidget(ui->mainPage);
  ui->deliveryPhoneEdit->clear();
  ui->pickupPhoneEdit->clear();
  returnTimer->stop();
}

void MainWindow::handlePhoneNumberSubmit()
{
  if (currentPage == DELIVERY_PHONE_PAGE) {
    QString phone = ui->deliveryPhoneEdit->text();
    if (validatePhoneNumber(phone, false)) {
      lastPhoneNumber = phone;
      switchToNextPage();
    } else {
      ui->deliveryPhoneEdit->clear();
      showErrorMessage("手机号输入有误，请检查");
    }
  } else if (currentPage == PICKUP_PHONE_PAGE) {
    QString shortPhone = ui->pickupPhoneEdit->text();
    if (validatePhoneNumber(shortPhone, true)) {
      lastPhoneNumber = shortPhone;
      // 验证手机号后四位是否匹配
      if (phoneNumberToCabinet.contains(shortPhone)) {
        // 找到对应的柜子
        selectedCabinetId = phoneNumberToCabinet[shortPhone];
        // 发送开门命令
        if (infoHub) {
          infoHub->sendDoorCommand(selectedCabinetId - 1);
          ui->boxOpenLabel->setText(QString("箱子%1已打开\n请取出物品后关门").arg(selectedCabinetId));
          switchToBoxOpen();
        }
      } else {
        ui->pickupPhoneEdit->clear();
        showErrorMessage("未找到对应的快递");
      }
    } else {
      ui->pickupPhoneEdit->clear();
      showErrorMessage("手机号输入有误，请检查");
    }
  }
}

bool MainWindow::validatePhoneNumber(const QString& phone, bool isShortPhone)
{
  QRegExp rx("[0-9]+");
  if (!rx.exactMatch(phone)) {
    return false;
  }

  if (isShortPhone) {
    return phone.length() == 4;
  } else {
    return phone.length() == 11 && phone.startsWith("1");
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
  QPixmap bgPixmap(":/images/gbx_agv.png");
  if (!bgPixmap.isNull()) {
    ui->mainPageBackground->setPixmap(bgPixmap.scaled(ui->mainPageBackground->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
  } else {
    ROS_ERROR_STREAM("Failed to load background image from path: :/images/gbx_agv.png");
  }
}

void MainWindow::shutdownPlugin()
{
  ROS_INFO_STREAM("Shutting down plugin");
  if (infoHub) {
    infoHub->requestInterruption();
    infoHub->wait();
  }
}

void MainWindow::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                              qt_gui_cpp::Settings& instance_settings) const
{
  // 保存设置
}

void MainWindow::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                 const qt_gui_cpp::Settings& instance_settings)
{
  // 恢复设置
}

} // namespace gbx_delivery_rqt

PLUGINLIB_EXPORT_CLASS(gbx_delivery_rqt::MainWindow, rqt_gui_cpp::Plugin)

