#pragma once

#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/logic/logic.hpp"
#include "crtp_cpp/packer/crtp_packer.hpp"
#include "crtp_cpp/packer/generic_commander_packer.hpp"
#include <vector>
#include <tuple>
#include <cmath>


/**
 * @brief Used for sending control setpoints to the Crazyflie
 */
class GenericCommanderLogic : public Logic {
public:
    /**
     * @brief Constructor for GenericCommanderLogic.
     * @param crtp_link A pointer to the CrtpLink object.
     */
    GenericCommanderLogic(CrtpLink* crtp_link);

    /**
     * @brief Sends a packet so that the priority of the current setpoint to the lowest non-disabled value,
     * so any new setpoint regardless of source will overwrite it.
     * @param remain_valid_milliseconds The remaining valid milliseconds.
     */
    void send_notify_setpoints_stop(uint32_t remain_valid_milliseconds = 0);

    /**
     * @brief Send STOP setpoing, stopping the motors and (potentially) falling.
     */
    void send_stop_setpoint();

    /**
     * @brief Send Velocity in the world frame of reference setpoint with yawrate commands
     * @param vx Velocity in x direction (m/s).
     * @param vy Velocity in y direction (m/s).
     * @param vz Velocity in z direction (m/s).
     * @param yawrate Yaw rate in degrees/s.
     */
    void send_velocity_world_setpoint(float vx, float vy, float vz, float yawrate);

    /**
     * @brief Control mode where the height is send as an absolute setpoint (intended
     * to be the distance to the surface under the Crazflie), while giving roll,
     * pitch and yaw rate commands
     * @param roll Roll in degrees.
     * @param pitch Pitch in degrees.
     * @param yawrate Yaw rate in degrees/s.
     * @param zdistance Distance in meters.
     */
    void send_zdistance_setpoint(float roll, float pitch, float yawrate, float zdistance);

    /**
     * @brief Control mode where the height is send as an absolute setpoint (intended
     * to be the distance to the surface under the Crazflie), while giving x, y velocity
     * commands in body-fixed coordinates.
     * @param vx Velocity in x direction (m/s).
     * @param vy Velocity in y direction (m/s).
     * @param yawrate Yaw rate in degrees/s.
     * @param zdistance Distance in meters.
     */
    void send_hover_setpoint(float vx, float vy, float yawrate, float zdistance);

    /**
     * @brief Control mode where the position, velocity, acceleration, orientation and angular
     * velocity are sent as absolute (world) values.
     * @param pos Position [x, y, z] in meters.
     * @param vel Velocity [vx, vy, vz] in m/s.
     * @param acc Acceleration [ax, ay, az] in m/s^2.
     * @param orientation Orientation [qx, qy, qz, qw] as quaternion components.
     * @param rollrate Roll rate in degrees/s.
     * @param pitchrate Pitch rate in degrees/s.
     * @param yawrate Yaw rate in degrees/s.
     */
    void send_full_state_setpoint(
        const std::vector<float>& pos, const std::vector<float>& vel, const std::vector<float>& acc,
        const std::vector<float>& orientation, float rollrate, float pitchrate, float yawrate);

    /**
     * @brief Control mode where the position is sent as absolute (world) x,y,z coordinate in
     * meter and the yaw is the absolute orientation.
     * @param x Position x in meters.
     * @param y Position y in meters.
     * @param z Position z in meters.
     * @param yaw Yaw in degrees.
     */
    void send_position_setpoint(float x, float y, float z, float yaw);

private:
    GenericCommanderPacker packer;

    uint32_t compress_quaternion(float const q[4]);

};

