#pragma once

#include "crtp_cpp/packer/crtp_packer.hpp"
#include <vector>
#include <cstdint>

class LocalizationPacker : public CrtpPacker {
public:
    LocalizationPacker();

    CrtpPacket send_extpos(float x, float y, float z);
    CrtpPacket send_short_lpp_packet(uint8_t dest_id, const std::vector<uint8_t>& lpp_data);
    CrtpPacket send_emergency_stop();
    CrtpPacket send_emergency_stop_watchdog();
    CrtpPacket send_extpose(float x, float y, float z, float qx, float qy, float qz, float qw);
    CrtpPacket send_lh_persist_data_packet(uint16_t mask_geo, uint16_t mask_calib);

protected:
    CrtpPacket prepare_packet(const std::vector<uint8_t>& data, uint8_t channel);
};