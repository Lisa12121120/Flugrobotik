#include "crtp_cpp/packer/hl_commander_packer.hpp"
#include <cstring> // For memcpy

#define PORT_HL_COMMANDER 0x08

#define CHANNEL_HL_COMMANDER 0

#define COMMAND_SET_GROUP_MASK 0
#define COMMAND_STOP 3
#define COMMAND_GO_TO 4
#define COMMAND_START_TRAJECTORY 5
#define COMMAND_DEFINE_TRAJECTORY 6
#define COMMAND_TAKEOFF_2 7
#define COMMAND_LAND_2 8

#define TRAJECTORY_LOCATION_MEM 1

HighLevelCommanderPacker::HighLevelCommanderPacker()
    : CrtpPacker(PORT_HL_COMMANDER) {}

CrtpPacket HighLevelCommanderPacker::prepare_packet(const std::vector<uint8_t>& data) 
{
    return CrtpPacker::prepare_packet(CHANNEL_HL_COMMANDER, data);
}

CrtpPacket HighLevelCommanderPacker::set_group_mask(uint8_t group_mask) {
    std::vector<uint8_t> data = {COMMAND_SET_GROUP_MASK, group_mask};
    return prepare_packet(data);
}

CrtpPacket HighLevelCommanderPacker::stop(uint8_t group_mask) {
    std::vector<uint8_t> data = {COMMAND_STOP, group_mask};
    return prepare_packet(data);
}

CrtpPacket HighLevelCommanderPacker::go_to(uint8_t group_mask, bool relative, float x, float y, float z, float yaw, float duration_s) {
    std::vector<uint8_t> data(1 + 1 + 1 + 4 + 4 + 4 + 4 + 4); // Pre-allocate

    data[0] = COMMAND_GO_TO;
    data[1] = group_mask;
    data[2] = relative;

    std::memcpy(&data[3], &x, 4);
    std::memcpy(&data[7], &y, 4);
    std::memcpy(&data[11], &z, 4);
    std::memcpy(&data[15], &yaw, 4);
    std::memcpy(&data[19], &duration_s, 4);

    return prepare_packet(data);
}

CrtpPacket HighLevelCommanderPacker::start_trajectory(uint8_t group_mask, bool relative, bool reversed, uint8_t trajectory_id, float time_scale) {
    std::vector<uint8_t> data(1 + 1 + 1 + 1 + 1 + 4);

    data[0] = COMMAND_START_TRAJECTORY;
    data[1] = group_mask;
    data[2] = relative;
    data[3] = reversed;
    data[4] = trajectory_id;
    std::memcpy(&data[5], &time_scale, 4);

    return prepare_packet(data);
}

CrtpPacket HighLevelCommanderPacker::define_trajectory(uint8_t trajectory_id, uint8_t type, uint32_t offset, uint8_t n_pieces) {
    std::vector<uint8_t> data(1 + 1 + 1 + 1 + 4 + 1);

    data[0] = COMMAND_DEFINE_TRAJECTORY;
    data[1] = trajectory_id;
    data[2] = TRAJECTORY_LOCATION_MEM;
    data[3] = type;
    std::memcpy(&data[4], &offset, 4);
    data[8] = n_pieces;

    return prepare_packet(data);
}

CrtpPacket HighLevelCommanderPacker::takeoff(uint8_t group_mask, float absolute_height_m, float target_yaw, bool use_current_yaw, float duration_s) {
    std::vector<uint8_t> data(1 + 1 + 4 + 4 + 1 + 4);

    data[0] = COMMAND_TAKEOFF_2;
    data[1] = group_mask;
    std::memcpy(&data[2], &absolute_height_m, 4);
    std::memcpy(&data[6], &target_yaw, 4);
    data[10] = use_current_yaw;
    std::memcpy(&data[11], &duration_s, 4);
        
    return prepare_packet(data);
}

CrtpPacket HighLevelCommanderPacker::land(uint8_t group_mask, float absolute_height_m, float target_yaw, bool use_current_yaw, float duration_s) {
    std::vector<uint8_t> data(1 + 1 + 4 + 4 + 1 + 4);

    data[0] = COMMAND_LAND_2;
    data[1] = group_mask;
    std::memcpy(&data[2], &absolute_height_m, 4);
    std::memcpy(&data[6], &target_yaw, 4);
    data[10] = use_current_yaw;
    std::memcpy(&data[11], &duration_s, 4);

    return prepare_packet(data);
}