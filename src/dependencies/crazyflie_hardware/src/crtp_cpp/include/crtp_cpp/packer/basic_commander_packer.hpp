#pragma once

#include "crtp_cpp/packer/crtp_packer.hpp"
#include <vector>
#include <cstdint>

class BasicCommanderPacker : public CrtpPacker {
public:
    BasicCommanderPacker();

    CrtpPacket send_setpoint(float roll, float pitch, float yawrate, uint16_t thrust);

protected:
    CrtpPacket prepare_packet(const std::vector<uint8_t>& data);
};