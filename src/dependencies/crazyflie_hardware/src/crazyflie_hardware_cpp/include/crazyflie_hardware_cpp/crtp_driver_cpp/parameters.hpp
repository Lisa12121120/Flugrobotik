

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/parameters_logic.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include "std_msgs/msg/empty.hpp"

class Parameters : public ParametersLogic
{
public:
    Parameters(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink *link);
    void initialize_parameters();

private:
    void download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg);

    rcl_interfaces::msg::SetParametersResult set_parameter_callback(const std::vector<rclcpp::Parameter> &parameters);

private:
    std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node;
    std::string logger_name;

    rclcpp::CallbackGroup::SharedPtr callback_group;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr downdload_toc_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr get_toc_info_sub;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle;
};