#pragma once

#include "crtp_cpp/packer/crtp_packer.hpp"
#include <vector>
#include <cstdint>

class GenericCommanderPacker : public CrtpPacker {
public:
    GenericCommanderPacker();

    CrtpPacket send_notify_sendpoints_stop(uint32_t remain_valid_milliseconds);
    CrtpPacket send_stop_setpoint();
    CrtpPacket send_velocity_world_setpoint(float vx, float vy, float vz, float yawrate);
    CrtpPacket send_zdistance_setpoint(float roll, float pitch, float yawrate, float zdistance);
    CrtpPacket send_hover_setpoint(float vx, float vy, float yawrate, float zdistance);
    CrtpPacket send_full_state_setpoint(int16_t x, int16_t y, int16_t z, int16_t vx, int16_t vy, int16_t vz, int16_t ax, int16_t ay, int16_t az, uint32_t orient_comp, int16_t rr, int16_t pr, int16_t yr);
    CrtpPacket send_position_setpoint(float x, float y, float z, float yaw);

protected:
    CrtpPacket prepare_packet(const std::vector<uint8_t>& data, uint8_t channel); // Added channel
};
