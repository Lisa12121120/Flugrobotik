

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/generic_commander_logic.hpp"
#include "crazyflie_interfaces/msg/notify_setpoints_stop.hpp"
#include "crazyflie_interfaces/msg/position.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class GenericCommander : public GenericCommanderLogic {
public:
    GenericCommander(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink * link);
private: 

    void notify_setpoints_stop_callback(const crazyflie_interfaces::msg::NotifySetpointsStop::SharedPtr msg);
    void cmd_position_callback(const crazyflie_interfaces::msg::Position::SharedPtr msg);
    

private: 
    std::string logger_name;

    rclcpp::CallbackGroup::SharedPtr callback_group; 

    rclcpp::Subscription<crazyflie_interfaces::msg::NotifySetpointsStop>::SharedPtr notify_setpoints_stop_sub;
    rclcpp::Subscription<crazyflie_interfaces::msg::Position>::SharedPtr cmd_position_sub;
};