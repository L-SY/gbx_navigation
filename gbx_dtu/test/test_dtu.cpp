//
// Created by lsy on 24-12-6.
//

// test/test_dtu.cpp
#include <ros/ros.h>
#include <gtest/gtest.h>
#include "gbx_dtu/EP_D200.h"
#include <navigation_msgs/IndoorDeliveryOrder.h>
#include <memory>

class DTUTest : public ::testing::Test {
protected:
  void SetUp() override {
    dtu_ = std::make_unique<EP_D200>();
    // 使用一个测试用的串口设备，你可能需要根据实际情况修改
    ASSERT_TRUE(dtu_->initializeSerial("/dev/ttyUSB0", 115200));
  }

  void TearDown() override {
    dtu_.reset();
  }

  std::unique_ptr<EP_D200> dtu_;
};

TEST_F(DTUTest, TestDeliveryOrderUpdate) {
  navigation_msgs::IndoorDeliveryOrder test_order;

  // 填充测试数据
  test_order.Number = "TEST001";
  test_order.RFID = "RFID123456";
  test_order.RFIDNumber = "RN789";
  test_order.ReceiverPhone = "13800138000";
  test_order.OrderNumber = 12345;
  test_order.ReceiverName = "测试接收者";
  test_order.SenderName = "测试发送者";
  test_order.Owner = "TEST_OWNER";
  test_order.Converted_RFID = "CONV_RFID_001";

  // 测试更新订单
  ASSERT_NO_THROW(dtu_->updateDeliveryOrder(test_order));

  // 测试发送数据
  ASSERT_TRUE(dtu_->sendData());
}

// 主函数运行所有测试
int main(int argc, char **argv) {
  ros::init(argc, argv, "test_dtu");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}