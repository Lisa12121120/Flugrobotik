#pragma once

#include "crtp_cpp/packer/crtp_packer.hpp"
#include <vector>
#include <cstdint>

class LinkLayerPacker : public CrtpPacker {
public:
    LinkLayerPacker();

    CrtpPacket echopacket();
    CrtpPacket sourcepacket();
    CrtpPacket sinkpacket();
    CrtpPacket nullpacket();
    CrtpPacket get_vbat();
    CrtpPacket platform_power_down();
    CrtpPacket stm_power_down();
    CrtpPacket stm_power_up();
    CrtpPacket reset_init();
    CrtpPacket reset_to_bootloader();
    CrtpPacket reset_to_firmware();

protected:
    CrtpPacket prepare_packet(uint8_t channel, const std::vector<uint8_t>& data);
};
