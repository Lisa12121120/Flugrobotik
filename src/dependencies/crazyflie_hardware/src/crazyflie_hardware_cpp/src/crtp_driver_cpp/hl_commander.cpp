#include "crazyflie_hardware_cpp/crtp_driver_cpp/hl_commander.hpp"
using std::placeholders::_1;

HighLevelCommander::HighLevelCommander(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink *link)
    : HighLevelCommanderLogic(link)
    , logger_name(node->get_name())
{
    callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    land_sub = node->create_subscription<crazyflie_interfaces::msg::Land>(
        "~/land",
        10,
        std::bind(&HighLevelCommander::land_callback, this, _1),
        sub_opt);

    takeoff_sub = node->create_subscription<crazyflie_interfaces::msg::Takeoff>(
        "~/takeoff",
        10,
        std::bind(&HighLevelCommander::takeoff_callback, this, _1),
        sub_opt);

    goto_sub = node->create_subscription<crazyflie_interfaces::msg::GoTo>(
        "~/go_to",
        10,
        std::bind(&HighLevelCommander::goto_callback, this, _1),
        sub_opt);

    RCLCPP_DEBUG(rclcpp::get_logger(logger_name), "High Level Commander initialized");
};

void HighLevelCommander::land_callback(const crazyflie_interfaces::msg::Land::SharedPtr msg)
{
    HighLevelCommanderLogic::send_land(msg->height, (double)(msg->duration.sec + msg->duration.nanosec * 1e-9), msg->group_mask, msg->yaw);
}

void HighLevelCommander::takeoff_callback(const crazyflie_interfaces::msg::Takeoff::SharedPtr msg)
{

    HighLevelCommanderLogic::send_takeoff(msg->height, (double)(msg->duration.sec + msg->duration.nanosec * 1e-9), msg->group_mask, msg->yaw);
}

void HighLevelCommander::goto_callback(const crazyflie_interfaces::msg::GoTo::SharedPtr msg)
{

    HighLevelCommanderLogic::send_go_to(msg->goal.x, msg->goal.y, msg->goal.z, msg->yaw, (double)(msg->duration.sec + msg->duration.nanosec * 1e-9), msg->relative, msg->group_mask);
}