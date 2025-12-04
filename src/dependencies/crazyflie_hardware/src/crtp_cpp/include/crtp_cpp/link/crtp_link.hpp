#pragma once

#include <stdint.h>
#include <vector>
#include <tuple>
#include <optional>
#include <functional>

struct CrtpPacket
{
  uint8_t port;
  uint8_t channel;
  uint8_t data[31];
  uint8_t data_length;

  bool expects_response;
  uint8_t matching_bytes;
  bool obeys_ordering;
};

struct CrtpRequest
{
  CrtpPacket packet;

  bool expects_response = false;
  uint8_t matching_bytes = 0;
  bool obeys_ordering;
};

class CrtpLink {
public:
  using CrtpCallbackType = std::function<void(const CrtpPacket&)>;

  CrtpLink(int channel, std::array<uint8_t, 5> address, int datarate);

  virtual void close_link() {};
  virtual void add_callback(uint8_t port, const CrtpCallbackType& callback) {};

  virtual void send_packet_no_response(CrtpRequest packet) {};
  virtual std::optional<CrtpPacket> send_packet(CrtpRequest request){}  ;
  virtual std::vector<CrtpPacket> send_batch_request(const std::vector<CrtpRequest> requests){} ;

protected:
  int channel;
  std::array<uint8_t, 5> address;
  int datarate;
};