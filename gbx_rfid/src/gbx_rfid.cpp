#include "gbx_rfid/gbx_rfid.h"
#include <sstream>
#include <iomanip>

namespace gbx_rfid {
GbxRfid::GbxRfid() : is_open_(false) {}

GbxRfid::~GbxRfid() {
  if (is_open_) {
    close();
  }
}

bool GbxRfid::init(const std::string& port, int baudrate) {
  try {
    if(!initSerial(port, baudrate)) {
      return false;
    }

    // 添加通信测试
    if(!testCommunication()) {
      ROS_ERROR("Communication test failed");
      return false;
    }

    if(!setRegion(CHINA_2_REGION)) {
      ROS_ERROR("Failed to set region to China 2");
      return false;
    }

    if(!checkConnection()) {
      return false;
    }

    is_open_ = true;

    std::vector<uint8_t> power_cmd = {
        0xBB, 0x00, 0xB6, 0x00, 0x02, 0x07, 0xD0, 0x8F, 0x7E
    };
    ROS_INFO_STREAM("Setting power command: " << commandToHexString(power_cmd));
    sendCommand(power_cmd);

    ROS_INFO("Successfully initialized RFID reader");
    return true;
  } catch(std::exception& e) {
    ROS_ERROR_STREAM("Exception in init: " << e.what());
    return false;
  }
}

bool GbxRfid::initSerial(const std::string port, int baudrate) {
  try {
    ser_.setPort(port);
    ser_.setBaudrate(baudrate);
        
    // 设置其他串口参数
    ser_.setBytesize(serial::eightbits);
    ser_.setParity(serial::parity_none);
    ser_.setStopbits(serial::stopbits_one);
    ser_.setFlowcontrol(serial::flowcontrol_none);
        
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser_.setTimeout(to);

    ROS_INFO("Attempting to open port %s at %d baud", port.c_str(), baudrate);
    ser_.open();
        
    if(ser_.isOpen()) {
      ROS_INFO("Serial port opened successfully");
      // 清空缓冲区
      ser_.flush();
      is_open_ = true;
      return true;
    } else {
      ROS_ERROR("Failed to open serial port");
      return false;
    }
  } catch(serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open RFID serial port " << port << ". Error: " << e.what());
    return false;
  } catch(std::exception& e) {
    ROS_ERROR_STREAM("Exception while opening port: " << e.what());
    return false;
  }
}

bool GbxRfid::testCommunication() {
  ROS_INFO("Testing communication...");
    
  // 发送获取功率的命令作为测试
  std::vector<uint8_t> test_cmd = {
      FRAME_HEAD,
      0x00,
      CMD_GET_POWER,
      0x00,
      0x00,
      0xB7,
      FRAME_END
  };
    
  ROS_INFO_STREAM("Sending test command: " << commandToHexString(test_cmd));
    
  if(!sendCommand(test_cmd)) {
    ROS_ERROR("Failed to send test command");
    return false;
  }
    
  std::vector<uint8_t> resp = readResponse(2000);  // 给更长的超时时间
  if(resp.empty()) {
    ROS_ERROR("No response to test command");
    return false;
  }
    
  ROS_INFO_STREAM("Test command response: " << commandToHexString(resp));
    
  return true;
}

bool GbxRfid::setRegion(uint8_t region) {
  ROS_INFO("Attempting to set region to: %d", region);
    
  std::vector<uint8_t> cmd = {
      FRAME_HEAD,
      0x00,
      CMD_SET_REGION,
      0x00,
      0x01,
      region,
      0x00,
      FRAME_END
  };

  uint8_t checksum = calculateChecksum(std::vector<uint8_t>(cmd.begin()+1, cmd.end()-2));
  cmd[cmd.size()-2] = checksum;

  ROS_INFO_STREAM("Sending region command: " << commandToHexString(cmd));

  if(!sendCommand(cmd)) {
    ROS_ERROR("Failed to send region command");
    return false;
  }

  std::vector<uint8_t> resp = readResponse();
    
  ROS_INFO_STREAM("Received region response: " << commandToHexString(resp));

  if(resp.empty()) {
    ROS_ERROR("No response received");
    return false;
  }

  if(resp.size() < 8) {
    ROS_ERROR("Response too short (size: %zu)", resp.size());
    return false;
  }

  if(resp[0] != FRAME_HEAD) {
    ROS_ERROR("Invalid frame head: 0x%02X", resp[0]);
    return false;
  }

  if(resp[2] != CMD_SET_REGION) {
    ROS_ERROR("Invalid command response: 0x%02X", resp[2]);
    return false;
  }

  if(resp[5] != RESP_OK) {
    ROS_ERROR("Region setting failed with response code: 0x%02X", resp[5]);
    return false;
  }

  if(resp[7] != FRAME_END) {
    ROS_ERROR("Invalid frame end: 0x%02X", resp[7]);
    return false;
  }

  ROS_INFO("Successfully set region");
  return true;
}

bool GbxRfid::checkConnection() {
  std::vector<uint8_t> cmd = {
      FRAME_HEAD,
      0x00,
      CMD_GET_POWER,
      0x00,
      0x00,
      0xB7,
      FRAME_END
  };

  ROS_INFO_STREAM("Sending connection check command: " << commandToHexString(cmd));

  if(!sendCommand(cmd)) {
    ROS_ERROR("Failed to send connection check command");
    return false;
  }

  std::vector<uint8_t> resp = readResponse();
    
  ROS_INFO_STREAM("Connection check response: " << commandToHexString(resp));

  if(resp.size() < 9 || resp[0] != FRAME_HEAD || resp[2] != CMD_GET_POWER || resp[8] != FRAME_END) {
    ROS_ERROR("Invalid response when checking connection");
    return false;
  }

  return true;
}

void GbxRfid::close() {
  if (is_open_) {
    ser_.close();
    is_open_ = false;
  }
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

  if(!sendCommand(cmd)) {
    ROS_ERROR("Failed to send read command");
    return false;
  }

  return true;
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

  if(resp[0] != FRAME_HEAD) {
    ROS_ERROR("Invalid frame head: 0x%02X", resp[0]);
    return false;
  }

  if(resp.back() != FRAME_END) {
    ROS_ERROR("Invalid frame end: 0x%02X", resp.back());
    return false;
  }

  if(resp[2] == 0xFF && resp[5] == 0x15) {
    ROS_DEBUG("No tag response detected");
    return false;
  }

  data.rssi = static_cast<int8_t>(resp[5]);
  ROS_DEBUG("RSSI: %d", data.rssi);

  std::string epc;
  for(size_t i = 8; i < resp.size()-3; i++) {
    char hex[3];
    snprintf(hex, sizeof(hex), "%02X", resp[i]);
    epc += hex;
  }
  ROS_DEBUG("Parsed EPC: %s", epc.c_str());

  data.epc = epc;
  data.timestamp = std::to_string(ros::Time::now().toSec());

  return true;
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
    return written == cmd.size();
  } catch(serial::IOException& e) {
    ROS_ERROR_STREAM("Failed to send command: " << e.what());
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
  ros::Time start_time = ros::Time::now();

  while(ros::Time::now() - start_time < ros::Duration(timeout_ms/1000.0)) {
    if(ser_.available()) {
      uint8_t byte;
      ser_.read(&byte, 1);
      response.push_back(byte);

      if(byte == FRAME_END && response.size() >= 7 && response[0] == FRAME_HEAD) {
        // 检查是否有足够的数据和校验和是否正确
        uint8_t expected_checksum = calculateChecksum(
            std::vector<uint8_t>(response.begin()+1, response.end()-2));
        if(expected_checksum == response[response.size()-2]) {
          ROS_DEBUG_STREAM("Valid response received: " << commandToHexString(response));
          return response;
        } else {
          ROS_WARN("Checksum mismatch: expected 0x%02X, got 0x%02X",
                   expected_checksum, response[response.size()-2]);
        }
      }
    }
    ros::Duration(0.001).sleep();
  }

  if(response.empty()) {
    ROS_WARN("No response received within timeout period (%d ms)", timeout_ms);
  } else {
    ROS_WARN_STREAM("Incomplete response received: " << commandToHexString(response));
  }

  return std::vector<uint8_t>();
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

  // 确保字符串长度为偶数
  if(hex_str.length() % 2 != 0) {
    ROS_ERROR("Invalid hex string length");
    return bytes;
  }

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

} // namespace gbx_rfid