

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/hl_commander_logic.hpp"
#include "crazyflie_interfaces/msg/takeoff.hpp"
#include "crazyflie_interfaces/msg/land.hpp"
#include "crazyflie_interfaces/msg/go_to.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


class HighLevelCommander : public HighLevelCommanderLogic {
public:
    HighLevelCommander(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink * link);
private: 

    void land_callback(const crazyflie_interfaces::msg::Land::SharedPtr msg);
    void takeoff_callback(const crazyflie_interfaces::msg::Takeoff::SharedPtr msg);
    void goto_callback(const crazyflie_interfaces::msg::GoTo::SharedPtr msg);
    

private: 
    std::string logger_name;

    rclcpp::CallbackGroup::SharedPtr callback_group; 

    rclcpp::Subscription<crazyflie_interfaces::msg::Land>::SharedPtr land_sub;
    rclcpp::Subscription<crazyflie_interfaces::msg::Takeoff>::SharedPtr takeoff_sub;
    rclcpp::Subscription<crazyflie_interfaces::msg::GoTo>::SharedPtr goto_sub;
};