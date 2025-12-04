#include "crtp_cpp/logic/localization_logic.hpp"
#include <algorithm>
#include <stdexcept>

LocalizationLogic::LocalizationLogic(CrtpLink* crtp_link)
    : Logic(crtp_link),
      packer(LocalizationPacker()) {}

void LocalizationLogic::send_extpos(const std::vector<float>& pos) {
    CrtpRequest request;
    request.packet = packer.send_extpos(pos[0], pos[1], pos[2]);
    link->send_packet_no_response(request);
}

void LocalizationLogic::send_short_lpp_packet(uint8_t dest_id, const std::vector<uint8_t>& data) {
    CrtpRequest request;
    request.packet = packer.send_short_lpp_packet(dest_id, data);
    link->send_packet_no_response(request);
}

void LocalizationLogic::send_emergency_stop() {
    CrtpRequest request;
    request.packet = packer.send_emergency_stop();
    link->send_packet_no_response(request);
}

void LocalizationLogic::send_emergency_stop_watchdog() {
    CrtpRequest request;
    request.packet = packer.send_emergency_stop_watchdog();
    link->send_packet_no_response(request);
}

void LocalizationLogic::send_extpose(const std::vector<float>& pos, const std::vector<float>& quat) {
    CrtpRequest request;
    request.packet = packer.send_extpose(pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]);
    link->send_packet_no_response(request);
}

void LocalizationLogic::send_lh_persist_data_packet(const std::vector<int>& geo_list, const std::vector<int>& calib_list) {
    std::vector<int> sorted_geo_list = geo_list;
    std::vector<int> sorted_calib_list = calib_list;
    std::sort(sorted_geo_list.begin(), sorted_geo_list.end());
    std::sort(sorted_calib_list.begin(), sorted_calib_list.end());

    int max_bs_nr = 15;
    if (!sorted_geo_list.empty()) {
        if (sorted_geo_list.front() < 0 || sorted_geo_list.back() > max_bs_nr) {
            throw std::runtime_error("Geometry BS list is not valid");
        }
    }
    if (!sorted_calib_list.empty()) {
        if (sorted_calib_list.front() < 0 || sorted_calib_list.back() > max_bs_nr) {
            throw std::runtime_error("Calibration BS list is not valid");
        }
    }

    uint16_t mask_geo = 0;
    uint16_t mask_calib = 0;
    for (int bs : sorted_geo_list) {
        mask_geo += 1 << bs;
    }
    for (int bs : sorted_calib_list) {
        mask_calib += 1 << bs;
    }

    CrtpRequest request;
    request.packet = packer.send_lh_persist_data_packet(mask_geo, mask_calib);
    link->send_packet_no_response(request);
}

std::map<std::string, std::vector<float>> LocalizationLogic::_decode_lh_angle(const std::vector<uint8_t>& data) {
    std::map<std::string, std::vector<float>> decoded_data;

    if (data.size() != 26) {
        throw std::runtime_error("Invalid data size for LH angle decoding");
    }

    float raw_data[9];
    std::memcpy(raw_data, data.data(), 26);

    decoded_data["basestation"] = {raw_data[0]};
    decoded_data["x"] = {raw_data[1], raw_data[1] - fp16_to_float(raw_data[2]), raw_data[1] - fp16_to_float(raw_data[3]), raw_data[1] - fp16_to_float(raw_data[4])};
    decoded_data["y"] = {raw_data[5], raw_data[5] - fp16_to_float(raw_data[6]), raw_data[5] - fp16_to_float(raw_data[7]), raw_data[5] - fp16_to_float(raw_data[8])};

    return decoded_data;
}

float LocalizationLogic::fp16_to_float(uint16_t fp16){
    uint32_t s = (fp16 >> 15) & 0x00000001; // sign
    uint32_t e = (fp16 >> 10) & 0x0000001f; // exponent
    uint32_t f = fp16 & 0x000003ff;         // fraction

    if (e == 0) {
        if (f == 0) {
            uint32_t result = s << 31;
            float float_result;
            std::memcpy(&float_result, &result, sizeof(float));
            return float_result;
        } else {
            while (!(f & 0x00000400)) {
                f <<= 1;
                e -= 1;
            }
            e += 1;
            f &= ~0x00000400;
        }
    } else if (e == 31) {
        if (f == 0) {
            uint32_t result = (s << 31) | 0x7f800000;
            float float_result;
            std::memcpy(&float_result, &result, sizeof(float));
            return float_result;
        } else {
            uint32_t result = (s << 31) | 0x7f800000 | (f << 13);
            float float_result;
            std::memcpy(&float_result, &result, sizeof(float));
            return float_result;
        }
    }

    e += 127 - 15;
    f <<= 13;
    uint32_t result = (s << 31) | (e << 23) | f;
    float float_result;
    std::memcpy(&float_result, &result, sizeof(float));
    return float_result;
}