#include "gbx_rfid/gbx_rfid.h"
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace gbx_rfid {

GbxRfid::GbxRfid() : is_open_(false) {}

GbxRfid::~GbxRfid() {
  if (is_open_) {
    close();
  }
}

bool GbxRfid::init(const std::string& port, int baudrate, const SerialConfig& config) {
  try {
    serial_config_ = config;
    if(!initSerial(port, baudrate, config)) {
      return false;
    }

    // 初始化后等待设备稳定
    ros::Duration(serial_config_.init_delay / 1000.0).sleep();

    // // 尝试通信测试
    // if(!testCommunication()) {
    //   ROS_ERROR("Communication test failed");
    //   return false;
    // }
    //
    // // 设置区域
    // if(!setRegion(CHINA_2_REGION)) {
    //   ROS_ERROR("Failed to set region to China 2");
    //   return false;
    // }

    is_open_ = true;

    // 设置功率
    std::vector<uint8_t> power_cmd = {
        0xBB, 0x00, 0xB6, 0x00, 0x02, 0x07, 0xD0, 0x8F, 0x7E
    };
    ROS_INFO_STREAM("Setting power command: " << commandToHexString(power_cmd));
    if (!sendCommand(power_cmd)) {
      ROS_WARN("Failed to set power, but continuing initialization");
    }

    ROS_INFO("Successfully initialized RFID reader");
    return true;
  } catch(const std::exception& e) {
    ROS_ERROR_STREAM("Exception in init: " << e.what());
    return false;
  }
}

bool GbxRfid::initSerial(const std::string port, int baudrate, const SerialConfig& config) {
  try {
    // 如果端口已经打开，先关闭
    if (ser_.isOpen()) {
      ser_.close();
    }

    ser_.setPort(port);
    ser_.setBaudrate(baudrate);

    // 设置串口参数
    ser_.setBytesize(serial::eightbits);
    ser_.setParity(serial::parity_none);
    ser_.setStopbits(serial::stopbits_one);
    ser_.setFlowcontrol(serial::flowcontrol_none);

    // 设置超时
    serial::Timeout to = serial::Timeout::simpleTimeout(config.timeout);
    to.read_timeout_constant = config.read_timeout;
    to.read_timeout_multiplier = 50;
    to.write_timeout_constant = config.write_timeout;
    to.write_timeout_multiplier = 50;
    ser_.setTimeout(to);

    ROS_INFO("Attempting to open port %s at %d baud", port.c_str(), baudrate);
    ser_.open();

    if(ser_.isOpen()) {
      ROS_INFO("Serial port opened successfully");

      // 清理缓冲区
      ser_.flush();
      while(ser_.available()) {
        ser_.read(ser_.available());
      }

      // 等待设备稳定
      ros::Duration(config.init_delay / 1000.0).sleep();

      is_open_ = true;
      return true;
    } else {
      ROS_ERROR("Failed to open serial port");
      return false;
    }
  } catch(const serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open RFID serial port " << port << ". Error: " << e.what());
    return false;
  } catch(const std::exception& e) {
    ROS_ERROR_STREAM("Exception while opening port: " << e.what());
    return false;
  }
}

bool GbxRfid::waitForValidResponse(std::vector<uint8_t>& response, int timeout_ms) {
  ros::Time start_time = ros::Time::now();
  response.clear();
  bool found_start = false;

  while(ros::Time::now() - start_time < ros::Duration(timeout_ms/1000.0)) {
    if(ser_.available()) {
      uint8_t byte;
      ser_.read(&byte, 1);

      // 寻找帧头
      if(!found_start) {
        if(byte == FRAME_HEAD) {
          found_start = true;
          response.push_back(byte);
        }
        continue;
      }

      response.push_back(byte);

      // 检查是否是完整的帧
      if(byte == FRAME_END && response.size() >= 7) {
        // 验证校验和
        uint8_t expected_checksum = calculateChecksum(
            std::vector<uint8_t>(response.begin()+1, response.end()-2));
        if(expected_checksum == response[response.size()-2]) {
          return true;
        } else {
          ROS_WARN("Checksum mismatch: expected 0x%02X, got 0x%02X",
                   expected_checksum, response[response.size()-2]);
          // 重新开始
          response.clear();
          found_start = false;
        }
      }

      // 如果数据太长，可能是错误的
      if(response.size() > 100) {
        ROS_WARN("Response too long, resetting");
        response.clear();
        found_start = false;
      }
    }
    ros::Duration(0.001).sleep();
  }

  ROS_WARN_STREAM("Timeout waiting for valid response after " << timeout_ms << "ms");
  return false;
}

bool GbxRfid::startReading(READ_MODE mode, int retry_count) {
  std::vector<uint8_t> cmd;

  if(mode == SINGLE_MODE) {
    cmd = {
        FRAME_HEAD,
        0x00,
        CMD_SINGLE_READ,
        0x00,
        0x00,
        0x22,
        FRAME_END
    };
  } else {
    uint8_t count_h = (retry_count >> 8) & 0xFF;
    uint8_t count_l = retry_count & 0xFF;

    cmd = {
        FRAME_HEAD,
        0x00,
        CMD_MULTI_READ,
        0x00,
        0x03,
        0x22,
        count_h,
        count_l,
        0x00,
        FRAME_END
    };

    cmd[8] = calculateChecksum(std::vector<uint8_t>(cmd.begin()+1, cmd.end()-2));
  }

  ROS_DEBUG_STREAM("Sending read command: " << commandToHexString(cmd));
  return sendCommand(cmd);
}

bool GbxRfid::stopReading() {
  std::vector<uint8_t> cmd = {
      FRAME_HEAD,
      0x00,
      CMD_STOP_MULTI,
      0x00,
      0x00,
      0x28,
      FRAME_END
  };

  ROS_DEBUG_STREAM("Sending stop command: " << commandToHexString(cmd));
  return sendCommand(cmd);
}

bool GbxRfid::parseResponse(const std::vector<uint8_t>& resp, RfidData& data) {
  ROS_DEBUG_STREAM("Parsing response: " << commandToHexString(resp));

  if(resp.size() < 7) {
    ROS_ERROR("Response too short (size: %zu)", resp.size());
    return false;
  }

  if(resp[0] != FRAME_HEAD || resp.back() != FRAME_END) {
    ROS_ERROR("Invalid frame markers");
    return false;
  }

  if(resp[2] == 0xFF && resp[5] == RESP_NO_TAG) {
    ROS_DEBUG("No tag response detected");
    return false;
  }

  data.rssi = static_cast<int8_t>(resp[5]);
  data.serial_config = serial_config_;

  std::string epc;
  for(size_t i = 8; i < resp.size()-3; i++) {
    char hex[3];
    snprintf(hex, sizeof(hex), "%02X", resp[i]);
    epc += hex;
  }

  data.epc = epc;
  data.timestamp = std::to_string(ros::Time::now().toSec());

  ROS_DEBUG("Parsed EPC: %s, RSSI: %d", epc.c_str(), data.rssi);
  return true;
}

bool GbxRfid::sendCommand(const std::vector<uint8_t>& cmd) {
  if(!is_open_) {
    ROS_ERROR("Serial port not open");
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  try {
    // 清空接收缓冲区
    while(ser_.available()) {
      ser_.read(ser_.available());
    }

    size_t written = ser_.write(cmd);
    bool success = (written == cmd.size());

    if (!success) {
      ROS_ERROR("Failed to write complete command");
    }

    return success;
  } catch(const serial::IOException& e) {
    ROS_ERROR_STREAM("Failed to send command: " << e.what());
    return false;
  } catch(const std::exception& e) {
    ROS_ERROR_STREAM("Exception while sending command: " << e.what());
    return false;
  }
}

bool GbxRfid::sendCommand(const std::string& cmd_hex) {
  return sendCommand(hexStringToBytes(cmd_hex));
}

std::vector<uint8_t> GbxRfid::readResponse(int timeout_ms) {
  if(!is_open_) {
    ROS_ERROR("Serial port not open while trying to read response");
    return std::vector<uint8_t>();
  }

  std::vector<uint8_t> response;
  if (!waitForValidResponse(response, timeout_ms)) {
    return std::vector<uint8_t>();
  }
  return response;
}

uint8_t GbxRfid::calculateChecksum(const std::vector<uint8_t>& data) {
  uint8_t sum = 0;
  for(uint8_t byte : data) {
    sum += byte;
  }
  return sum;
}

std::vector<uint8_t> GbxRfid::hexStringToBytes(const std::string& hex) {
  std::vector<uint8_t> bytes;
  std::string hex_str = hex;

  // 移除所有空格
  hex_str.erase(std::remove(hex_str.begin(), hex_str.end(), ' '), hex_str.end());

  try {
    for(size_t i = 0; i < hex_str.length(); i += 2) {
      std::string byteString = hex_str.substr(i, 2);
      bytes.push_back(static_cast<uint8_t>(std::stoi(byteString, nullptr, 16)));
    }
  } catch(const std::exception& e) {
    ROS_ERROR_STREAM("Error converting hex string: " << e.what());
    return std::vector<uint8_t>();
  }

  return bytes;
}

std::string GbxRfid::bytesToHexString(const std::vector<uint8_t>& bytes) {
  std::stringstream ss;
  ss << std::hex << std::setfill('0');

  for(size_t i = 0; i < bytes.size(); ++i) {
    ss << std::setw(2) << static_cast<int>(bytes[i]);
    if(i < bytes.size() - 1) {
      ss << " ";
    }
  }

  return ss.str();
}

std::string GbxRfid::commandToHexString(const std::vector<uint8_t>& cmd) {
  if(cmd.empty()) {
    return "[]";
  }

  std::stringstream ss;
  ss << "[";
  for(size_t i = 0; i < cmd.size(); ++i) {
    ss << "0x" << std::hex << std::setfill('0') << std::setw(2)
       << static_cast<int>(cmd[i]);
    if(i < cmd.size() - 1) {
      ss << " ";
    }
  }
  ss << "]";
  return ss.str();
}

bool GbxRfid::getLatestData(RfidData& data) {
  std::vector<uint8_t> resp = readResponse();
  if(resp.empty()) {
    ROS_DEBUG("No response received");
    return false;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if(parseResponse(resp, data)) {
    ROS_DEBUG("Parsed tag data - EPC: %s, RSSI: %d", data.epc.c_str(), data.rssi);
    latest_data_ = data;
    return true;
  }

  ROS_DEBUG("Failed to parse response");
  return false;
}

void GbxRfid::close() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (is_open_) {
    try {
      ser_.close();
    } catch(const std::exception& e) {
      ROS_WARN_STREAM("Exception while closing port: " << e.what());
    }
    is_open_ = false;
  }
}

} // namespace gbx_rfid