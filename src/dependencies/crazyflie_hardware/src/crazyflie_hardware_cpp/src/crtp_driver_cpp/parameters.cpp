#include "crazyflie_hardware_cpp/crtp_driver_cpp/parameters.hpp"
using std::placeholders::_1;

Parameters::Parameters(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink *link)
    : ParametersLogic(link, std::string("mein_pfad"))
    , node(node)
    , logger_name(node->get_name())
{
    callback_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group;

    downdload_toc_sub = node->create_subscription<std_msgs::msg::Empty>(
        "~/download_parameters_toc",
        10,
        std::bind(&Parameters::download_toc_callback, this, _1),
        sub_opt);

    get_toc_info_sub = node->create_subscription<std_msgs::msg::Empty>(
        "~/get_parameters_toc_info",
        10,
        std::bind(&Parameters::get_toc_info_callback, this, _1),
        sub_opt);

    RCLCPP_DEBUG(rclcpp::get_logger(logger_name), "Parameters  initialized");
}

void Parameters::initialize_parameters()
{
    this->initialize_toc(); // Load toc from cf or from file

    for (const auto &entry : ParametersLogic::toc_entries)
    {
        auto group = entry.group;
        auto name = entry.name;
        std::ostringstream ss;
        ss << group << "." << name;
        if (entry.isInteger())
        {
            if (auto node_shared = node.lock()) node_shared->declare_parameter(ss.str(), rclcpp::PARAMETER_INTEGER);
        }
        else if (entry.isDouble())
        {
            if (auto node_shared = node.lock()) node_shared->declare_parameter(ss.str(), rclcpp::PARAMETER_DOUBLE);
        }
    }
    if (auto node_shared = node.lock()) {
        param_callback_handle = node_shared->add_on_set_parameters_callback(std::bind(&Parameters::set_parameter_callback, this, std::placeholders::_1));    
    }
}

rcl_interfaces::msg::SetParametersResult Parameters::set_parameter_callback(const std::vector<rclcpp::Parameter> &parameters)
{
    // This gets called if a parameter gets set. We want to set it on the crazyflie as well.

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters)
    {
        std::string param_name = param.get_name();
        size_t dot = param_name.find('.');

        if (dot != std::string::npos)
        {
            std::string group = param_name.substr(0, dot);
            std::string name = param_name.substr(dot + 1);

            if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                result.successful = send_set_parameter(group, name, std::variant<int, double>((int)param.as_int()));
            }
            else if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
                result.successful = send_set_parameter(group, name, std::variant<int, double>(param.as_double()));
            }
        }

        if (!result.successful)
            return result;
    }
    return result;
}

void Parameters::download_toc_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    ParametersLogic::send_download_toc_items();
    ParametersLogic::write_to_file();

    // auto [nbr_of_items, crc] = ParametersLogic::send_get_toc_info();
    // bool success = ParametersLogic::load_from_file(crc);
    // RCLCPP_WARN(node->get_logger(), "%d", success);
}

void Parameters::get_toc_info_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
    auto [nbr_of_items, crc] = ParametersLogic::send_get_toc_info();
    RCLCPP_WARN(rclcpp::get_logger(logger_name), "%d, %X", nbr_of_items, crc);
}