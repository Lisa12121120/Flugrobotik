#include "crazyflie_hardware_cpp/crtp_driver_cpp/logblock.hpp"

LogBlock::LogBlock(
    std::shared_ptr<rclcpp::node_interfaces::NodeBaseInterface> node_base_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTopicsInterface> node_topics_interface, 
    std::shared_ptr<rclcpp::node_interfaces::NodeLoggingInterface> node_logging_interface,
    std::shared_ptr<rclcpp::node_interfaces::NodeTimersInterface> node_timers_interface,
    std::shared_ptr<rclcpp::CallbackGroup> callback_group,
    const std::string name,
    StartBlockMethod start_block_method,
    StopBlockMethod stop_block_method
) 
: m_base_interface(node_base_interface)
, m_logging_interface(node_logging_interface)
, m_timers_interface(node_timers_interface)
, m_callback_group(callback_group)
, m_block_name(name)
, m_start_block_method(start_block_method)
, m_stop_block_method(stop_block_method)
{
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    std::string ns = "~/log/" + name + "/";

    m_start_log_block_sub = rclcpp::create_subscription<std_msgs::msg::Int16>(
        node_topics_interface,
        ns + "start",
        10,
        std::bind(&LogBlock::m_start_log_block, this, std::placeholders::_1),
        sub_opt);

    m_stop_log_block_sub = rclcpp::create_subscription<std_msgs::msg::Empty>(
        node_topics_interface,
        ns + "stop",
        10,
        std::bind(&LogBlock::m_stop_log_block, this, std::placeholders::_1),
        sub_opt);
    
    auto publisher_options = rclcpp::PublisherOptions();
    publisher_options.callback_group = callback_group;
    m_log_data_publisher = rclcpp::create_publisher<crazyflie_interfaces::msg::GenericLogData>(
        node_topics_interface,
        ns + "data",
        rclcpp::QoS(10),
        publisher_options
    );
}

void LogBlock::m_publish_log_data(std::vector<double>& data)
{
    auto log_msg = crazyflie_interfaces::msg::GenericLogData();
    log_msg.values = data;

    m_log_data_publisher->publish(log_msg);
}

void LogBlock::m_start_log_block(const std::shared_ptr<std_msgs::msg::Int16> msg)
{
    RCLCPP_INFO(m_logging_interface->get_logger(), "Starting LogBlock %s with period %d d10ms", m_block_name.c_str(), msg->data);
    m_start_block_method(msg->data);
}

void LogBlock::m_stop_log_block(const std::shared_ptr<std_msgs::msg::Empty> msg)
{
    (void)msg;
    RCLCPP_INFO(m_logging_interface->get_logger(), "Stopping LogBlock %s", m_block_name.c_str());
    m_stop_block_method();
}