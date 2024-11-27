//
// Created by lsy-cloude3.5 Sonnet on 24-11-25.
//

#pragma once
#define FRAME_HEAD 0xBB
#define FRAME_END  0x7E
#include <ros/ros.h>
#include <serial/serial.h>
#include <string>
#include <vector>

namespace gbx_rfid
{
enum CMD_CODE {
  CMD_SET_REGION     = 0x07,
  CMD_GET_POWER      = 0xB7,
  CMD_SET_POWER      = 0xB6,
  CMD_SINGLE_READ    = 0x22,
  CMD_MULTI_READ     = 0x27,
  CMD_STOP_MULTI     = 0x28,
};

enum REGION_CODE {
  CHINA_2_REGION    = 0x01,
};

enum RESPONSE_CODE {
  RESP_OK           = 0x00,
  RESP_FAIL         = 0xFF,
  RESP_NO_TAG       = 0x15,
};

enum READ_MODE {
  SINGLE_MODE = 0,
  MULTI_MODE  = 1
};

struct RfidData {
  std::string epc;
  int rssi;
  std::string timestamp;
};

class GbxRfid {
public:
  GbxRfid();
  ~GbxRfid();

  bool init(const std::string& port = "/dev/ttyUSB0",
            int baudrate = 115200);

  bool initSerial(const std::string port, int baudrate);
  bool setRegion(uint8_t region);
  bool checkConnection();

  void close();

// Read
  bool startReading(READ_MODE mode = SINGLE_MODE, int retry_count = 1);
  bool stopReading();
  bool getLatestData(RfidData& data);

//  Send
  bool sendCommand(const std::vector<uint8_t>& cmd);
  bool sendCommand(const std::string& cmd_hex);
  std::vector<uint8_t> readResponse(int timeout_ms = 1000);

private:
  bool parseResponse(const std::vector<uint8_t>& resp, RfidData& data);
  uint8_t calculateChecksum(const std::vector<uint8_t>& data);
  std::vector<uint8_t> hexStringToBytes(const std::string& hex);
  std::string bytesToHexString(const std::vector<uint8_t>& bytes);

  serial::Serial ser_;
  bool is_open_;
  std::mutex mutex_;
  RfidData latest_data_;
};
} // namespace gbx_rfid