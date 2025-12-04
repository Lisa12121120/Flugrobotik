

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/console_logic.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"

class Console : public ConsoleLogic
{
public:
    Console(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink *link);

private:
    void console_message(const std::string);
    void crtp_response_callback(const CrtpPacket &packet) override;

private:
    std::string logger_name;

    rclcpp::CallbackGroup::SharedPtr callback_group;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr console_publisher;
};