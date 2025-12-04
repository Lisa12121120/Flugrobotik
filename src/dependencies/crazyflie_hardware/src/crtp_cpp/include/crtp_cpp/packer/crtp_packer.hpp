#pragma once

#include <crtp_cpp/link/crtp_link.hpp>
#include <stdint.h>
#include <vector>

class CrtpPacker {
public:
  CrtpPacker(int port);
  virtual ~CrtpPacker() = default;

  CrtpPacket prepare_packet(int channel, const std::vector<uint8_t>& data);

protected:
  int port;
};