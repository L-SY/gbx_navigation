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

void MainWindow::setupConnections()
{
  // 基本页面切换
  connect(ui->pickupButton, &QPushButton::clicked, this, &MainWindow::switchToPickupMode);
  connect(ui->deliveryButton, &QPushButton::clicked, this, &MainWindow::switchToDeliveryMode);

  // 箱门状态变化
  connect(this, &MainWindow::boxOpened, this, &MainWindow::switchToBoxOpen);
  connect(this, &MainWindow::boxClosed, this, &MainWindow::switchToDoorClosed);
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
  QPixmap mapPixmap(":/images/2F_whole.png");
  if (!mapPixmap.isNull()) {
    mapLabel->setPixmap(mapPixmap.scaled(800, 600, Qt::KeepAspectRatio, Qt::SmoothTransformation));
    mapLabel->setAlignment(Qt::AlignCenter);
  } else {
    qDebug() << "Failed to load map image from path:" << ":/images/2F_whole.png";
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
  QHBoxLayout* buttonLayout = new QHBoxLayout(buttonContainer);
  buttonLayout->setSpacing(15);  // 按钮之间的间距

  // 创建区域按钮
  QStringList areas = {"A区", "B区", "C区", "D区", "E区", "F区", "G区"};
  for (const QString& area : areas) {
    QPushButton* button = new QPushButton(area, buttonContainer);
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

    connect(button, &QPushButton::clicked, this, [this, &area, areas]() {
      int index = areas.indexOf(area);
      if (index != -1) {
        handleDestinationSelect(index + 1);
      }
    });

    buttonLayout->addWidget(button);
  }

  mainLayout->addWidget(buttonContainer);

  // 设置布局的拉伸因子
  mainLayout->setStretchFactor(mapContainer, 3);
  mainLayout->setStretchFactor(buttonContainer, 1);

  // 更新界面
  ui->destinationPage->update();
}

void MainWindow::createDestinationMarkers()
{
  // 清除之前的标记
  qDeleteAll(destinationMarkers);  // 使用 qDeleteAll 代替手动循环删除
  destinationMarkers.clear();
  destinationPositions.clear();

  // 计算标记的位置（在一行中均匀分布）
  int mapWidth = ui->mapLabel->width();
  int mapHeight = ui->mapLabel->height();
  int spacing = mapWidth / (NUM_DESTINATIONS + 1);
  int y = mapHeight / 2;

  for (int i = 0; i < NUM_DESTINATIONS; i++) {
    int x = spacing * (i + 1);
    QPoint pos(x, y);

    // 创建并保存标记
    QLabel* marker = createStarMarker(pos);
    marker->setParent(ui->mapLabel);
    marker->show();

    // 为标记添加鼠标事件
    marker->installEventFilter(this);
    marker->setObjectName(QString("destination_%1").arg(i));

    destinationMarkers.append(marker);
    destinationPositions.append(pos);
  }
}

QLabel* MainWindow::createStarMarker(const QPoint& pos)
{
  QLabel* marker = new QLabel(ui->mapLabel);

  // 创建一个五角星SVG图像
  QString svgContent = QString(
      "<svg width='30' height='30'>"
      "<path d='M15,0 L18.5,10.5 L30,10.5 L21,17 L24.5,27.5 L15,21 L5.5,27.5 L9,17 L0,10.5 L11.5,10.5 Z' "
      "fill='red' stroke='black' stroke-width='1'/>"
      "</svg>"
  );

  QSvgRenderer renderer(svgContent.toUtf8());
  QPixmap pixmap(30, 30);
  pixmap.fill(Qt::transparent);
  QPainter painter(&pixmap);
  renderer.render(&painter);

  marker->setPixmap(pixmap);
  marker->setGeometry(pos.x() - 15, pos.y() - 15, 30, 30);  // 居中显示
  marker->setCursor(Qt::PointingHandCursor);  // 鼠标悬停时显示手型光标

  return marker;
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
  // 处理标记的点击事件
  for (int i = 0; i < destinationMarkers.size(); i++) {
    if (obj == destinationMarkers[i]) {
      if (event->type() == QEvent::MouseButtonPress) {
        handleDestinationSelect(i + 1);  // 使用 i+1 作为目的地ID
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
//  TODO: Just for test
  handleDoorStateChange(false, 1);
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
    if (validatePhoneNumber(phone, false)) {
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
  if (!isOpen) {  // 门已关闭
    if (currentMode == DELIVERY) {
      // 寄件模式：门关闭后跳转到选择目的地
      switchToDestination();
    } else {
      // 取件模式：门关闭后跳转到感谢页面
      switchToDoorClosed();
    }
  }
}

void MainWindow::updateDoorStatus(int boxId, bool isOpen)
{
  isOpen = false;
  // TODO: 更新箱门状态显示
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