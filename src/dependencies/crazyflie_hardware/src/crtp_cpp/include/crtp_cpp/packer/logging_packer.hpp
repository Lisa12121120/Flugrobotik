#pragma once

#include "crtp_cpp/packer/toc_packer.hpp"
#include <vector>
#include <cstdint>
#include <tuple>

#define PORT_LOGGING 5

// Channels used for the logging port
#define TOC_CHANNEL 0
#define CONTROL_CHANNEL 1
#define LOGDATA_CHANNEL 2


enum LogType {
  LogTypeUint8  = 0x01,
  LogTypeUint16 = 0x02, 
  LogTypeUint32 = 0x03, 
  LogTypeInt8   = 0x04,
  LogTypeInt16  = 0x05,
  LogTypeInt32  = 0x06,
  LogTypeFloat  = 0x07,
  LogTypeFP16   = 0x08,
};

union LogValue {
  uint8_t valueUint8;
  uint16_t valueUint16;
  uint32_t valueUint32;
  int8_t  valueInt8;
  int16_t valueInt16;
  int32_t valueInt32;
  float valueFloat;
  float valueFP16;
};

class LoggingPacker : public TocPacker {
public:
 

  LoggingPacker(); // Constructor

  CrtpRequest create_block(uint8_t block_id, const std::vector<std::pair<uint8_t, uint16_t>>& variables);
  CrtpRequest start_block(uint8_t index, uint8_t period);
  CrtpRequest stop_block(uint8_t index);

  CrtpRequest reset();


private:
  std::vector<uint8_t> create_block_content(const std::vector<std::pair<uint8_t, uint16_t>>& variables);

protected:
    CrtpPacket prepare_control_packet(const std::vector<uint8_t>& data);
};
