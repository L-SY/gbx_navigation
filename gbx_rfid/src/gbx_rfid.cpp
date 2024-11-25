//
// Created by lsy on 24-11-25.
//
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
    sendCommand(power_cmd);

    ROS_INFO("Successfully initialized RFID reader");
    return true;
  } catch(serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open RFID serial port " << port << ". Error: " << e.what());
    return false;
  }
}

bool GbxRfid::initSerial(const std::string port, int baudrate){
  try {
    ser_.setPort(port);
    ser_.setBaudrate(baudrate);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser_.setTimeout(to);
    ser_.open();
    is_open_ = true;
    ROS_INFO("Successfully opened RFID serial port");
    return true;
  } catch(serial::IOException& e) {
    ROS_ERROR_STREAM("Unable to open RFID serial port " << port << ". Error: " << e.what());
    return false;
  }
}

bool GbxRfid::setRegion(uint8_t region) {
  // 命令格式: BB 00 07 00 01 01 09 7E
  std::vector<uint8_t> cmd = {
      FRAME_HEAD,
      0x00,
      CMD_SET_REGION,
      0x00,
      0x01,
      region,
      0x00,    // 校验位先置0
      FRAME_END
  };

  // 计算校验和 - 从第2个字节到倒数第3个字节的累加和的低字节
  uint8_t checksum = 0;
  for(size_t i = 1; i < cmd.size()-2; i++) {
    checksum += cmd[i];
  }
  cmd[cmd.size()-2] = checksum;  // 更新校验位

  if(!sendCommand(cmd)) {
    return false;
  }

  std::vector<uint8_t> resp = readResponse();
  if(resp.size() < 8 || resp[0] != FRAME_HEAD || resp[2] != CMD_SET_REGION ||
      resp[5] != RESP_OK || resp[7] != FRAME_END) {
    ROS_ERROR("Invalid response when setting region");
    return false;
  }

  return true;
}

// 检查连接是否正常
bool GbxRfid::checkConnection() {
  // 发送获取功率命令: BB 00 B7 00 00 B7 7E
  std::vector<uint8_t> cmd = {
      FRAME_HEAD,
      0x00,
      CMD_GET_POWER,
      0x00,
      0x00,
      0xB7,
      FRAME_END
  };

  if(!sendCommand(cmd)) {
    return false;
  }

  std::vector<uint8_t> resp = readResponse();
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
    // 多次读卡命令: BB 00 27 00 03 22 XX XX YY 7E
    // XX XX为读取次数(最大FFFF)，YY为校验
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

  if(!sendCommand(cmd)) {
    ROS_ERROR("Failed to send read command");
    return false;
  }

  ROS_DEBUG("Successfully sent read command");
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

  return sendCommand(cmd);
}

bool GbxRfid::parseResponse(const std::vector<uint8_t>& resp, RfidData& data) {
    ROS_DEBUG("Response size: %zu", resp.size());

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

    ROS_DEBUG("Command byte: 0x%02X", resp[2]);

    // 打印完整响应内容
    std::string debug_hex;
    for(uint8_t byte : resp) {
      char hex[4];
      snprintf(hex, sizeof(hex), "%02X ", byte);
      debug_hex += hex;
    }
    ROS_DEBUG("Full response: %s", debug_hex.c_str());

    if(resp[2] == 0xFF && resp[5] == 0x15) {
      ROS_DEBUG("No tag response detected");
      return false;
    }

//    if(resp[2] != CMD_SINGLE_READ && resp[2] != CMD_MULTI_READ) {
//      ROS_ERROR("Unexpected command byte: 0x%02X", resp[2]);
//      return false;
//    }

    data.rssi = static_cast<int8_t>(resp[5]);
    ROS_DEBUG("RSSI: %d", data.rssi);

    // 打印解析的EPC
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
//  if(resp.size() < 7 || resp[0] != FRAME_HEAD || resp.back() != FRAME_END) {
//    return false;
//  }
//
//  // 如果是无卡响应: BB 01 FF 00 01 15 16 7E
//  if(resp[2] == 0xFF && resp[5] == 0x15) {
//    return false;
//  }
//
//  // 解析EPC号 - 正常响应格式:
//  // BB 02 22 00 11 DC 30 00 E2 80 68 94 00 00 50 24 58 95 B5 EB D5 F9 6E 7E
//  //    |  |  |  |  |  |  |  |------------EPC-------------|    CRC    |  |
//  if(resp[2] == CMD_SINGLE_READ || resp[2] == CMD_MULTI_READ) {
//    // RSSI值
//    data.rssi = static_cast<int8_t>(resp[5]);
//
//    // PC值
//    uint16_t pc = (resp[6] << 8) | resp[7];
//
//    // EPC号 - 从第9个字节开始,去掉最后3个字节(CRC和帧尾)
//    std::string epc;
//    for(size_t i = 8; i < resp.size()-3; i++) {
//      char hex[3];
//      snprintf(hex, sizeof(hex), "%02X", resp[i]);
//      epc += hex;
//    }
//    data.epc = epc;
//
//    // 时间戳
//    data.timestamp = std::to_string(ros::Time::now().toSec());
//
//    return true;
//  }
//
//  return false;
//}

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
        uint8_t checksum = calculateChecksum(
            std::vector<uint8_t>(response.begin()+1, response.end()-2));
        if(checksum == response[response.size()-2]) {
          return response;
        }
      }
    }
    ros::Duration(0.001).sleep();
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

  hex_str.erase(std::remove(hex_str.begin(), hex_str.end(), ' '), hex_str.end());

  for(size_t i = 0; i < hex_str.length(); i += 2) {
    std::string byteString = hex_str.substr(i, 2);
    uint8_t byte = (uint8_t)strtol(byteString.c_str(), NULL, 16);
    bytes.push_back(byte);
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




} //namespace gbx_rfid
