#include "crtp_cpp/packer/logging_packer.hpp"
#include <tuple>
#include <cstring> // Add this line


// Commands used when accessing the Log configurations
#define CMD_CREATE_BLOCK 0
#define CMD_APPEND_BLOCK 1
#define CMD_DELETE_BLOCK 2
#define CMD_START_LOGGING 3
#define CMD_STOP_LOGGING 4
#define CMD_RESET_LOGGING 5
#define CMD_CREATE_BLOCK_V2 6
#define CMD_APPEND_BLOCK_V2 7

LoggingPacker::LoggingPacker()
    : TocPacker(PORT_LOGGING) {}


CrtpPacket LoggingPacker::prepare_control_packet(const std::vector<uint8_t>& data)
{
  return CrtpPacker::prepare_packet(CONTROL_CHANNEL, data);
}


CrtpRequest LoggingPacker::create_block(uint8_t block_id, const std::vector<std::pair<uint8_t, uint16_t>>& variables) {
  CrtpRequest request;
  std::vector<uint8_t> data = {CMD_CREATE_BLOCK_V2, block_id};

  std::vector<uint8_t> content_data = create_block_content(variables);
  data.insert(data.end(), content_data.begin(), content_data.end());
  request.packet = prepare_control_packet(data);
  request.expects_response = true;
  request.matching_bytes = 2;
  return request;
}

CrtpRequest LoggingPacker::start_block(uint8_t index, uint8_t period) {
  CrtpRequest request;
  std::vector<uint8_t> data = {CMD_START_LOGGING, index, period};
  request.packet = prepare_control_packet(data);
  request.expects_response = true;
  request.matching_bytes = 2;
  return request;
}

CrtpRequest LoggingPacker::stop_block(uint8_t index) {
  CrtpRequest request;
  std::vector<uint8_t> data = {CMD_STOP_LOGGING, index};
  request.packet = prepare_control_packet(data);
  request.expects_response = true;
  request.matching_bytes = 2;
  return request;
}

CrtpRequest LoggingPacker::reset() {
    CrtpRequest request;

    std::vector<uint8_t> data = {CMD_RESET_LOGGING};
    request.packet = prepare_control_packet(data);
    request.expects_response = true;
    request.matching_bytes = 1;
    return request;
}


std::vector<uint8_t> LoggingPacker::create_block_content(const std::vector<std::pair<uint8_t, uint16_t>>& variables) {
  std::vector<uint8_t> data;
  for (const auto& var : variables) {
    uint8_t type = var.first;
    uint16_t var_id = var.second;

    data.push_back(type);
    data.push_back(static_cast<uint8_t>(var_id & 0xFF));       // Lower byte
    data.push_back(static_cast<uint8_t>((var_id >> 8) & 0xFF)); // Higher byte
  }
  return data;
}
