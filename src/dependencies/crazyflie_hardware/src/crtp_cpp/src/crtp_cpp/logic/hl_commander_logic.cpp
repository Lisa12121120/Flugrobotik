#include "crtp_cpp/logic/hl_commander_logic.hpp"

HighLevelCommanderLogic::HighLevelCommanderLogic(
        CrtpLink * crtp_link
) : Logic(crtp_link),
    packer(HighLevelCommanderPacker())
{}

void HighLevelCommanderLogic::send_set_group_mask(int group_mask) {
    CrtpRequest request;
    request.packet = packer.set_group_mask(group_mask);
    link->send_packet_no_response(request);
}

void HighLevelCommanderLogic::send_stop(int group_mask) {
    CrtpRequest request;
    request.packet = packer.stop(group_mask);
    link->send_packet_no_response(request);
}

void HighLevelCommanderLogic::send_go_to(double x, double y, double z, double yaw, double duration_s, bool relative, int group_mask) {
    CrtpRequest request;
    request.packet = packer.go_to(group_mask, relative, x, y, z, yaw, duration_s);
    link->send_packet_no_response(request);
}

void HighLevelCommanderLogic::send_start_trajectory(int trajectory_id, double time_scale, bool relative, bool reversed, int group_mask) {
    CrtpRequest request;
    request.packet = packer.start_trajectory(group_mask, relative, reversed, trajectory_id, time_scale);
    link->send_packet_no_response(request);
}

void HighLevelCommanderLogic::send_define_trajectory(int trajectory_id, int offset, int n_pieces, int type) {
    CrtpRequest request;
    request.packet = packer.define_trajectory(trajectory_id, type, offset, n_pieces);
    link->send_packet_no_response(request);
}

void HighLevelCommanderLogic::send_takeoff(double absolute_height_m, double duration_s, int group_mask, double yaw) {
    double target_yaw = yaw;
    bool useCurrentYaw = false;
    if (yaw == 0.0) { // Check if yaw is default value which is 0.0
        target_yaw = 0.0;
        useCurrentYaw = true;
    }
    CrtpRequest request;
    request.packet = packer.takeoff(group_mask, absolute_height_m, target_yaw, useCurrentYaw, duration_s);
    link->send_packet_no_response(request);
}

void HighLevelCommanderLogic::send_land(double absolute_height_m, double duration_s, int group_mask, double yaw) {
    double target_yaw = yaw;
    bool useCurrentYaw = false;
    if (yaw == 0.0) { // Check if yaw is default value which is 0.0
        target_yaw = 0.0;
        useCurrentYaw = true;
    }
    CrtpRequest request;
    request.packet = packer.land(group_mask, absolute_height_m, target_yaw, useCurrentYaw, duration_s);
    link->send_packet_no_response(request);
}