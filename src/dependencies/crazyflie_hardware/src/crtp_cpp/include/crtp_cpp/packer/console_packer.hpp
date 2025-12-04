#pragma once

#include "crtp_cpp/packer/crtp_packer.hpp"
#include <vector>
#include <cstdint>

#define PORT_CONSOLE 0
#define CHANNEL_CONSOLE 0

class ConsolePacker : public CrtpPacker
{
public:
    ConsolePacker();

    CrtpPacket consolepacket();

protected:
    CrtpPacket prepare_packet(const std::vector<uint8_t> &data);
};