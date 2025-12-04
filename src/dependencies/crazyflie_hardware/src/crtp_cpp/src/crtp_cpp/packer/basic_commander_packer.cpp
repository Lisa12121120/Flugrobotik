#include "crtp_cpp/packer/basic_commander_packer.hpp"
#include <cstring>

#define PORT_BASIC_COMMANDER 0x03
#define CHANNEL 0x0

BasicCommanderPacker::BasicCommanderPacker()
    : CrtpPacker(PORT_BASIC_COMMANDER) {}

CrtpPacket BasicCommanderPacker::prepare_packet(const std::vector<uint8_t>& data) {
    return CrtpPacker::prepare_packet(CHANNEL, data);
}

CrtpPacket BasicCommanderPacker::send_setpoint(float roll, float pitch, float yawrate, uint16_t thrust) {
    std::vector<uint8_t> data(4 + 4 + 4 + 2);

    std::memcpy(&data[0], &roll, 4);
    std::memcpy(&data[4], &pitch, 4);
    std::memcpy(&data[8], &yawrate, 4);
    std::memcpy(&data[12], &thrust, 2);

    return prepare_packet(data);
}
