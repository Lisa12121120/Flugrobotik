#include "crazyflie_hardware_cpp/crtp_driver_cpp/console.hpp"
using std::placeholders::_1;

Console::Console(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink *link)
    : ConsoleLogic(link)
    , logger_name(node->get_name())

{
    callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    console_publisher = node->create_publisher<std_msgs::msg::String>(
        "~/console",
        10);

    for (int i = 0; i < 5; i++) ConsoleLogic::send_consolepacket(); // This initializes communication with the Crazyflie.
    RCLCPP_DEBUG(rclcpp::get_logger(logger_name), "Console initialized");
}

void Console::crtp_response_callback(const CrtpPacket &packet)
{
    static std::string message("");
    // A message can be from multiple messages.
    // A \n ends one message.
    if (packet.channel == CHANNEL_CONSOLE)
    {
        std::string string((char *)packet.data, packet.data_length);

        message.append(string);

        if (message.back() == '\n' || message.back() == '\r')
        {
            message.pop_back(); // Do not print newline twice.
            RCLCPP_WARN(rclcpp::get_logger(logger_name), "%s", message.c_str());
            this->console_message(message);
            message.clear();
        }
    }
}

void Console::console_message(const std::string message)
{
    auto msg = std_msgs::msg::String();
    msg.data = message;
    console_publisher->publish(msg);
}