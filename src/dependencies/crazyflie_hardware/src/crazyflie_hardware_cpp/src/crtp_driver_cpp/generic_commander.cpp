#include "crazyflie_hardware_cpp/crtp_driver_cpp/generic_commander.hpp"
using std::placeholders::_1;

GenericCommander::GenericCommander(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink *link)
    : GenericCommanderLogic(link)
    , logger_name(node->get_name())
{
    callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    notify_setpoints_stop_sub = node->create_subscription<crazyflie_interfaces::msg::NotifySetpointsStop>(
        "~/notify_setpoints_stop",
        10,
        std::bind(&GenericCommander::notify_setpoints_stop_callback, this, _1),
        sub_opt);

    cmd_position_sub = node->create_subscription<crazyflie_interfaces::msg::Position>(
        "~/cmd_position",
        10,
        std::bind(&GenericCommander::cmd_position_callback, this, _1),
        sub_opt);
        
    RCLCPP_DEBUG(rclcpp::get_logger(logger_name), "Generic Commander initialized");
};

void GenericCommander::notify_setpoints_stop_callback(const crazyflie_interfaces::msg::NotifySetpointsStop::SharedPtr msg)
{
    GenericCommanderLogic::send_notify_setpoints_stop(msg->remain_valid_millisecs);
}

void GenericCommander::cmd_position_callback(const crazyflie_interfaces::msg::Position::SharedPtr msg)
{
    GenericCommanderLogic::send_position_setpoint(msg->x, msg->y, msg->z, msg->yaw);
}