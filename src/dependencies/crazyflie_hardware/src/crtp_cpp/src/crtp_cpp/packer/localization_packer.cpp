
#include "crtp_cpp/packer/localization_packer.hpp"
#include <cstring>

#define PORT_LOCALIZATION 6

#define POSITION_CH 0
#define GENERIC_CH 1

#define RANGE_STREAM_REPORT 0
#define RANGE_STREAM_REPORT_FP16 1
#define LPS_SHORT_LPP_PACKET 2
#define EMERGENCY_STOP 3
#define EMERGENCY_STOP_WATCHDOG 4
#define COMM_GNSS_NMEA 6
#define COMM_GNSS_PROPRIETARY 7
#define EXT_POSE 8
#define EXT_POSE_PACKED 9
#define LH_ANGLE_STREAM 10
#define LH_PERSIST_DATA 11

LocalizationPacker::LocalizationPacker()
    : CrtpPacker(PORT_LOCALIZATION) {}

CrtpPacket LocalizationPacker::prepare_packet(const std::vector<uint8_t>& data, uint8_t channel) {
    return CrtpPacker::prepare_packet(channel, data);
}

CrtpPacket LocalizationPacker::send_extpos(float x, float y, float z) {
    std::vector<uint8_t> data(4 * 3); // 3 floats
    std::memcpy(&data[0], &x, 4);
    std::memcpy(&data[4], &y, 4);
    std::memcpy(&data[8], &z, 4);
    return prepare_packet(data, POSITION_CH);
}

CrtpPacket LocalizationPacker::send_short_lpp_packet(uint8_t dest_id, const std::vector<uint8_t>& lpp_data) {
    std::vector<uint8_t> data;
    data.push_back(LPS_SHORT_LPP_PACKET);
    data.push_back(dest_id);
    data.insert(data.end(), lpp_data.begin(), lpp_data.end());
    return prepare_packet(data, GENERIC_CH);
}

CrtpPacket LocalizationPacker::send_emergency_stop() {
    std::vector<uint8_t> data = {EMERGENCY_STOP};
    return prepare_packet(data, GENERIC_CH);
}

CrtpPacket LocalizationPacker::send_emergency_stop_watchdog() {
    std::vector<uint8_t> data = {EMERGENCY_STOP_WATCHDOG};
    return prepare_packet(data, GENERIC_CH);
}

CrtpPacket LocalizationPacker::send_extpose(float x, float y, float z, float qx, float qy, float qz, float qw) {
    std::vector<uint8_t> data(1 + 4 * 7); // 1 byte type, 7 floats
    data[0] = EXT_POSE;
    std::memcpy(&data[1], &x, 4);
    std::memcpy(&data[5], &y, 4);
    std::memcpy(&data[9], &z, 4);
    std::memcpy(&data[13], &qx, 4);
    std::memcpy(&data[17], &qy, 4);
    std::memcpy(&data[21], &qz, 4);
    std::memcpy(&data[25], &qw, 4);
    return prepare_packet(data, GENERIC_CH);
}

CrtpPacket LocalizationPacker::send_lh_persist_data_packet(uint16_t mask_geo, uint16_t mask_calib) {
    std::vector<uint8_t> data(1 + 2 + 2); // 1 byte type, 2 shorts
    data[0] = LH_PERSIST_DATA;
    std::memcpy(&data[1], &mask_geo, 2);
    std::memcpy(&data[3], &mask_calib, 2);
    return prepare_packet(data, GENERIC_CH);
}
