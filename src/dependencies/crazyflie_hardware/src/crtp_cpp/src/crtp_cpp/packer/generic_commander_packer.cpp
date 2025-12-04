#include "crtp_cpp/packer/generic_commander_packer.hpp"
#include <cstring>


#define PORT_COMMANDER_GENERIC 0x07

#define SET_SETPOINT_CHANNEL 0
#define META_COMMAND_CHANNEL 1

#define TYPE_STOP 0
#define TYPE_VELOCITY_WORLD 1
#define TYPE_ZDISTANCE 2
#define TYPE_HOVER 5
#define TYPE_FULL_STATE 6
#define TYPE_POSITION 7

#define TYPE_META_COMMAND_NOTIFY_SETPOINTS_STOP 0

GenericCommanderPacker::GenericCommanderPacker()
    : CrtpPacker(PORT_COMMANDER_GENERIC) {}

CrtpPacket GenericCommanderPacker::prepare_packet(const std::vector<uint8_t>& data, uint8_t channel) {
    return CrtpPacker::prepare_packet(channel, data);
}

CrtpPacket GenericCommanderPacker::send_notify_sendpoints_stop(uint32_t remain_valid_milliseconds) {
    std::vector<uint8_t> data(1 + 4);
    data[0] = TYPE_META_COMMAND_NOTIFY_SETPOINTS_STOP;
    std::memcpy(&data[1], &remain_valid_milliseconds, 4);
    return prepare_packet(data, META_COMMAND_CHANNEL);
}

CrtpPacket GenericCommanderPacker::send_stop_setpoint() {
    std::vector<uint8_t> data = {TYPE_STOP};
    return prepare_packet(data, SET_SETPOINT_CHANNEL);
}

CrtpPacket GenericCommanderPacker::send_velocity_world_setpoint(float vx, float vy, float vz, float yawrate) {
    std::vector<uint8_t> data(1 + 4 + 4 + 4 + 4);
    data[0] = TYPE_VELOCITY_WORLD;
    std::memcpy(&data[1], &vx, 4);
    std::memcpy(&data[5], &vy, 4);
    std::memcpy(&data[9], &vz, 4);
    std::memcpy(&data[13], &yawrate, 4);
    return prepare_packet(data, SET_SETPOINT_CHANNEL);
}

CrtpPacket GenericCommanderPacker::send_zdistance_setpoint(float roll, float pitch, float yawrate, float zdistance) {
    std::vector<uint8_t> data(1 + 4 + 4 + 4 + 4);
    data[0] = TYPE_ZDISTANCE;
    std::memcpy(&data[1], &roll, 4);
    std::memcpy(&data[5], &pitch, 4);
    std::memcpy(&data[9], &yawrate, 4);
    std::memcpy(&data[13], &zdistance, 4);
    return prepare_packet(data, SET_SETPOINT_CHANNEL);
}

CrtpPacket GenericCommanderPacker::send_hover_setpoint(float vx, float vy, float yawrate, float zdistance) {
    std::vector<uint8_t> data(1 + 4 + 4 + 4 + 4);
    data[0] = TYPE_HOVER;
    std::memcpy(&data[1], &vx, 4);
    std::memcpy(&data[5], &vy, 4);
    std::memcpy(&data[9], &yawrate, 4);
    std::memcpy(&data[13], &zdistance, 4);
    return prepare_packet(data, SET_SETPOINT_CHANNEL);
}

CrtpPacket GenericCommanderPacker::send_full_state_setpoint(int16_t x, int16_t y, int16_t z, int16_t vx, int16_t vy, int16_t vz, int16_t ax, int16_t ay, int16_t az, uint32_t orient_comp, int16_t rr, int16_t pr, int16_t yr) {
    std::vector<uint8_t> data(1 + 2 + 2 + 2 + 2 + 2 + 2 + 2 + 2 + 2 + 4 + 2 + 2 + 2);
    data[0] = TYPE_FULL_STATE;
    std::memcpy(&data[1], &x, 2);
    std::memcpy(&data[3], &y, 2);
    std::memcpy(&data[5], &z, 2);
    std::memcpy(&data[7], &vx, 2);
    std::memcpy(&data[9], &vy, 2);
    std::memcpy(&data[11], &vz, 2);
    std::memcpy(&data[13], &ax, 2);
    std::memcpy(&data[15], &ay, 2);
    std::memcpy(&data[17], &az, 2);
    std::memcpy(&data[19], &orient_comp, 4);
    std::memcpy(&data[23], &rr, 2);
    std::memcpy(&data[25], &pr, 2);
    std::memcpy(&data[27], &yr, 2);
    return prepare_packet(data, SET_SETPOINT_CHANNEL);
}

CrtpPacket GenericCommanderPacker::send_position_setpoint(float x, float y, float z, float yaw) {
    std::vector<uint8_t> data(1 + 4 + 4 + 4 + 4);
    data[0] = TYPE_POSITION;
    std::memcpy(&data[1], &x, 4);
    std::memcpy(&data[5], &y, 4);
    std::memcpy(&data[9], &z, 4);
    std::memcpy(&data[13], &yaw, 4);
    return prepare_packet(data, SET_SETPOINT_CHANNEL);
}