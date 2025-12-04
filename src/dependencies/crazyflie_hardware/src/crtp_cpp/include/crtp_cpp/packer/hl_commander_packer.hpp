#pragma once

#include "crtp_cpp/packer/crtp_packer.hpp"
#include <memory>
#include <functional>
#include <vector>
#include <cstdint> // For uint8_t

class HighLevelCommanderPacker : public CrtpPacker {
public:
    HighLevelCommanderPacker();

    CrtpPacket set_group_mask(uint8_t group_mask);
    CrtpPacket stop(uint8_t group_mask);
    CrtpPacket go_to(uint8_t group_mask, bool relative, float x, float y, float z, float yaw, float duration_s);
    CrtpPacket start_trajectory(uint8_t group_mask, bool relative, bool reversed, uint8_t trajectory_id, float time_scale);
    CrtpPacket define_trajectory(uint8_t trajectory_id, uint8_t type, uint32_t offset, uint8_t n_pieces);
    CrtpPacket takeoff(uint8_t group_mask, float absolute_height_m, float target_yaw, bool use_current_yaw, float duration_s);
    CrtpPacket land(uint8_t group_mask, float absolute_height_m, float target_yaw, bool use_current_yaw, float duration_s);

protected:
    CrtpPacket prepare_packet(const std::vector<uint8_t>& data);
};