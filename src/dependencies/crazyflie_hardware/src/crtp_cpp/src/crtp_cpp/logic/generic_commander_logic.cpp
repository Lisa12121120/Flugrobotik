#include "crtp_cpp/logic/generic_commander_logic.hpp"

GenericCommanderLogic::GenericCommanderLogic(CrtpLink* crtp_link)
    : Logic(crtp_link),
      packer(GenericCommanderPacker()) {}

void GenericCommanderLogic::send_notify_setpoints_stop(uint32_t remain_valid_milliseconds) {
    CrtpRequest request;
    request.packet = packer.send_notify_sendpoints_stop(remain_valid_milliseconds);
    link->send_packet_no_response(request);
}

void GenericCommanderLogic::send_stop_setpoint() {
    CrtpRequest request;
    request.packet = packer.send_stop_setpoint();
    link->send_packet_no_response(request);
}

void GenericCommanderLogic::send_velocity_world_setpoint(float vx, float vy, float vz, float yawrate) {
    CrtpRequest request;
    request.packet = packer.send_velocity_world_setpoint(vx, vy, vz, yawrate);
    link->send_packet_no_response(request);
}

void GenericCommanderLogic::send_zdistance_setpoint(float roll, float pitch, float yawrate, float zdistance) {
    CrtpRequest request;
    request.packet = packer.send_zdistance_setpoint(roll, pitch, yawrate, zdistance);
    link->send_packet_no_response(request);
}

void GenericCommanderLogic::send_hover_setpoint(float vx, float vy, float yawrate, float zdistance) {
    CrtpRequest request;
    request.packet = packer.send_hover_setpoint(vx, vy, yawrate, zdistance);
    link->send_packet_no_response(request);
}

void GenericCommanderLogic::send_full_state_setpoint(
    const std::vector<float>& pos, const std::vector<float>& vel, const std::vector<float>& acc,
    const std::vector<float>& orientation, float rollrate, float pitchrate, float yawrate) {

    auto vector_to_mm_16bit = [](const std::vector<float>& vec) {
        return std::make_tuple(static_cast<int16_t>(vec[0] * 1000), static_cast<int16_t>(vec[1] * 1000), static_cast<int16_t>(vec[2] * 1000));
    };

    int16_t x, y, z;
    std::tie(x, y, z) = vector_to_mm_16bit(pos);
    int16_t vx, vy, vz;
    std::tie(vx, vy, vz) = vector_to_mm_16bit(vel);
    int16_t ax, ay, az;
    std::tie(ax, ay, az) = vector_to_mm_16bit(acc);
    int16_t rr, pr, yr;
    std::tie(rr, pr, yr) = vector_to_mm_16bit({rollrate, pitchrate, yawrate});
    
    float q[4] = { orientation[0], orientation[1], orientation[2], orientation[3] };
    uint32_t orient_comp = compress_quaternion(q); 

    CrtpRequest request;
    request.packet = packer.send_full_state_setpoint(x, y, z, vx, vy, vz, ax, ay, az, orient_comp, rr, pr, yr);
    link->send_packet_no_response(request);
}

void GenericCommanderLogic::send_position_setpoint(float x, float y, float z, float yaw) {
    CrtpRequest request;
    request.packet = packer.send_position_setpoint(x, y, z, yaw);
    link->send_packet_no_response(request);
}

uint32_t GenericCommanderLogic::compress_quaternion(float const q[4]) {
    // we send the values of the quaternion's smallest 3 elements.
    unsigned i_largest = 0;
    for (unsigned i = 1; i < 4; ++i) {
        if (fabsf(q[i]) > fabsf(q[i_largest])) {
        i_largest = i;
        }
    }

    // since -q represents the same rotation as q,
    // transform the quaternion so the largest element is positive.
    // this avoids having to send its sign bit.
    unsigned negate = q[i_largest] < 0;

    // 1/sqrt(2) is the largest possible value
    // of the second-largest element in a unit quaternion.

    // do compression using sign bit and 9-bit precision per element.
    uint32_t comp = i_largest;
    for (unsigned i = 0; i < 4; ++i) {
        if (i != i_largest) {
        unsigned negbit = (q[i] < 0) ^ negate;
        unsigned mag = ((1 << 9) - 1) * (fabsf(q[i]) / (float)M_SQRT1_2) + 0.5f;
        comp = (comp << 10) | (negbit << 9) | mag;
        }
    }

    return comp;
}