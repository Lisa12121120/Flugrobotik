#pragma once

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/logging_logic.hpp"

#include "crazyflie_hardware_cpp/crtp_driver_cpp/logblock.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "crazyflie_interfaces/msg/generic_log_data.hpp"
#include "crazyflie_interfaces/msg/pose_stamped_array.hpp"
#include "crazyflie_interfaces/msg/log_block.hpp"

#include "std_msgs/msg/empty.hpp"

class Logging : public LoggingLogic {
public:
    Logging(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink * link);

    void initialize_logging();

    void start_logging_pm();
    void start_logging_pose();

private: 

    void download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg);

    void m_create_log_block(const crazyflie_interfaces::msg::LogBlock::SharedPtr msg);

    void crtp_response_callback(const CrtpPacket&  packet) override; 

    void m_start_logging_block(int block_id, int period_ms_d10)
    {
        LoggingLogic::start_block(block_id, period_ms_d10);
    }
    void m_stop_logging_block(int block_id)
    {
        LoggingLogic::stop_block(block_id);
    }


private: 
    std::string logger_name;
    std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node;


    rclcpp::CallbackGroup::SharedPtr callback_group; 

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr downdload_toc_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr get_toc_info_sub;
    rclcpp::Subscription<crazyflie_interfaces::msg::LogBlock>::SharedPtr m_create_log_block_sub;

    bool log_state;
    bool log_pose;
    rclcpp::Publisher<crazyflie_interfaces::msg::GenericLogData>::SharedPtr log_state_pub;
    rclcpp::Publisher<crazyflie_interfaces::msg::PoseStampedArray>::SharedPtr log_pose_pub;

    uint8_t next_log_block_id = 2;
    std::map<int, std::shared_ptr<LogBlock>> m_log_blocks;
};  