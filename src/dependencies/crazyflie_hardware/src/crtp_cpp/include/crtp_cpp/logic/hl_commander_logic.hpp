#pragma once

#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/logic/logic.hpp"
#include "crtp_cpp/packer/crtp_packer.hpp"
#include "crtp_cpp/packer/hl_commander_packer.hpp"
#include <memory>
#include <functional>

#define ALL_GROUPS 0

#define TRAJECTORY_TYPE_POLY4D 0
#define TRAJECTORY_TYPE_POLY4D_COMPRESSED 1

/**
 * @brief Logic for high-level commander commands.
 */
class HighLevelCommanderLogic : public Logic {
public:
    /**
     * @brief Constructor for HighLevelCommanderLogic.
     * @param crtp_link A pointer to the CrtpLink object.
     */
    HighLevelCommanderLogic(CrtpLink* crtp_link);

    /**
     * @brief Set the group mask that the Crazyflie belongs to.
     * @param group_mask Mask for which groups this CF belongs to.
     */
    void send_set_group_mask(int group_mask = ALL_GROUPS);

    /**
     * @brief Stops the current trajectory (turns off the motors).
     * @param group_mask Mask for which CFs this should apply to.
     */
    void send_stop(int group_mask = ALL_GROUPS);

    /**
     * @brief Go to an absolute or relative position.
     * @param x X (m).
     * @param y Y (m).
     * @param z Z (m).
     * @param yaw Yaw (radians).
     * @param duration_s Time it should take to reach the position (s).
     * @param relative True if x, y, z is relative to the current position.
     * @param group_mask Mask for which CFs this should apply to.
     */
    void send_go_to(double x, double y, double z, double yaw, double duration_s, bool relative = false, int group_mask = ALL_GROUPS);

    /**
     * @brief Starts executing a specified trajectory.
     * @param trajectory_id Id of the trajectory (previously defined by define_trajectory).
     * @param time_scale Time factor; 1.0 = original speed; >1.0: slower; <1.0: faster.
     * @param relative Set to True, if trajectory should be shifted to current setpoint.
     * @param reversed Set to True, if trajectory should be executed in reverse.
     * @param group_mask Mask for which CFs this should apply to.
     */
    void send_start_trajectory(int trajectory_id, double time_scale = 1.0, bool relative = false, bool reversed = false, int group_mask = ALL_GROUPS);

    /**
     * @brief Define a trajectory that has previously been uploaded to memory.
     * @param trajectory_id The id of the trajectory.
     * @param offset Offset in uploaded memory.
     * @param n_pieces Nr of pieces in the trajectory.
     * @param type The type of trajectory data; TRAJECTORY_TYPE_POLY4D or TRAJECTORY_TYPE_POLY4D_COMPRESSED.
     */
    void send_define_trajectory(int trajectory_id, int offset, int n_pieces, int type = TRAJECTORY_TYPE_POLY4D);

    /**
     * @brief Vertical takeoff from current x-y position to given height.
     * @param absolute_height_m Absolute (m).
     * @param duration_s Time it should take until target height is reached (s).
     * @param group_mask Mask for which CFs this should apply to.
     * @param yaw Yaw (rad). Use current yaw if set to 0.0.
     */
    void send_takeoff(double absolute_height_m, double duration_s, int group_mask = ALL_GROUPS, double yaw = 0.0);

    /**
     * @brief Vertical land from current x-y position to given height.
     * @param absolute_height_m Absolute (m).
     * @param duration_s Time it should take until target height is reached (s).
     * @param group_mask Mask for which CFs this should apply to.
     * @param yaw Yaw (rad). Use current yaw if set to 0.0.
     */
    void send_land(double absolute_height_m, double duration_s, int group_mask = ALL_GROUPS, double yaw = 0.0);

private:
    HighLevelCommanderPacker packer;
};
