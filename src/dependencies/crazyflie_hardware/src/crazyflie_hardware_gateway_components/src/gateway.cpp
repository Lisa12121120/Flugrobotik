#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp/node_interfaces/node_base.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "rclcpp_components/node_factory.hpp"

#include "ament_index_cpp/get_resource.hpp"
#include "rcpputils/split.hpp"
#include "class_loader/class_loader.hpp"


#include "crazyflie_hardware_gateway_interfaces/srv/add_crazyflie.hpp"
#include "crazyflie_hardware_gateway_interfaces/srv/remove_crazyflie.hpp"

#include <memory>
#include <filesystem>
class GatewayException : public std::runtime_error
{
public:
  explicit GatewayException(const std::string & error_desc)
  : std::runtime_error(error_desc) {}
};

struct DedicatedExecutorWrapper
  {
    std::shared_ptr<rclcpp::Executor> executor;
    std::thread thread;
    std::atomic_bool thread_initialized;

    /// Constructor for the wrapper.
    /// This is necessary as atomic variables don't have copy/move operators
    /// implemented so this structure is not copyable/movable by default
    explicit DedicatedExecutorWrapper(std::shared_ptr<rclcpp::Executor> exec)
    : executor(exec),
      thread_initialized(false)
    {
    }
  };

class CrazyflieGateway : public rclcpp::Node
{
public:
  CrazyflieGateway(
    std::weak_ptr<rclcpp::Executor> executor,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("crazyflie_hardware_gateway", options)
  , executor_(executor)
  , crazyflies_()
  {
    auto service_qos = rmw_qos_profile_services_default;
    service_qos.depth = 100; // This way it is possible to queue up multiple add requestst

    add_service_ = this->create_service<crazyflie_hardware_gateway_interfaces::srv::AddCrazyflie>(
      "~/add_crazyflie", std::bind(&CrazyflieGateway::handle_add_crazyflie, this, std::placeholders::_1, std::placeholders::_2),
      service_qos);

    remove_service_ = this->create_service<crazyflie_hardware_gateway_interfaces::srv::RemoveCrazyflie>(
      "~/remove_crazyflie", std::bind(&CrazyflieGateway::handle_remove_crazyflie, this, std::placeholders::_1, std::placeholders::_2),
      service_qos);
    
    cleanup_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&CrazyflieGateway::cleanup_callback, this));
  
    factory_ = create_component_factory("crazyflie_hardware_cpp", "CrazyflieNode");

    RCLCPP_INFO(get_logger(), "Crazyflie Gateway ready.");
  }

private: 
  std::pair<bool, std::string> add_crazyflie(int id, 
    int channel, 
    const geometry_msgs::msg::Point & initial_position,
    const std::string & type)
  {
    RCLCPP_INFO(get_logger(), "Adding Crazyflie with Channel: %d, Id: %d", channel, id);
    std::pair<uint8_t, uint8_t> crazyflie_key = {id, channel};

    if (crazyflies_.count(crazyflie_key))
    {
      return std::make_pair(false, "Crazyflie with id '" + std::to_string(id) + "' already exists.");
    }

    auto options = create_node_options(id, channel, initial_position, type);
    try {
      auto node = factory_->create_node_instance(options);
      auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
      exec->add_node(node.get_node_base_interface());

      auto entry = crazyflies_.emplace(std::make_pair(crazyflie_key, std::make_pair(node, exec)));

      DedicatedExecutorWrapper & wrapper = entry.first->second.second;
      wrapper.executor = exec;

      auto & thread_initialized = wrapper.thread_initialized;
      wrapper.thread = std::thread(
        [exec, &thread_initialized ]() {
          thread_initialized = true;
          try {
            exec->spin();
          } catch (...) {}
        }
      );
      // // Downcast to lifecycle node interface
      auto lifecycle_node = std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(node.get_node_instance());
      if (!lifecycle_node) {
         throw std::runtime_error("Failed to cast to LifecycleNodeInterface");
       }
      // // Transition to configure
       auto ret = lifecycle_node.get()->configure();    
       if (ret.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) 
       {
          remove_crazyflie(id, channel);
          return std::make_pair(false, "Configuration failed.");
       }

    } catch (const std::exception & ex) {
      // In the case that the component constructor throws an exception,
      // rethrow into the following catch block.
      throw GatewayException(
              "Component constructor threw an exception: " + std::string(ex.what()));
    } catch (...) {
      // In the case that the component constructor throws an exception,
      // rethrow into the following catch block.
      throw GatewayException("Component constructor threw an exception");
    }
    
    return std::make_pair(true, "Success!");
  }

  std::pair<bool, std::string> remove_crazyflie(int id, int channel)
  {
    RCLCPP_INFO(get_logger(), "Removing Crazyflie with Channel: %d, Id: %d", channel, id);
    std::pair<uint8_t, uint8_t> crazyflie_key = {id, channel};
    auto cf = crazyflies_.find(crazyflie_key);
    if (cf == crazyflies_.end()) {
      return std::make_pair(false,
        "Couldn't remove Crazyflie with Channel: " + std::to_string(channel) 
        + ", ID: " + std::to_string(id) + "; not in list.");
    }
    
    if (!cf->second.second.thread_initialized)
    {
      rclcpp::sleep_for(std::chrono::milliseconds(1)); // This only happens when add and removed are called near simultaniously. 
    }
    cf->second.second.executor->cancel();
    cf->second.second.thread.join();
    crazyflies_.erase(cf);
    return std::make_pair(true, "Success!");
  }

private:
  void cleanup_callback()
  {
    for (auto& [key, entry] : crazyflies_ )
    {
      auto lifecycle_node = std::static_pointer_cast<rclcpp_lifecycle::LifecycleNode>(entry.first.get_node_instance());
      if (lifecycle_node) 
      {
        if (lifecycle_node.get()->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
        {
          remove_crazyflie(key.first, key.second);
          return;
        }
      }
    }
  }

  void handle_add_crazyflie(const std::shared_ptr<crazyflie_hardware_gateway_interfaces::srv::AddCrazyflie::Request> request,
                            std::shared_ptr<crazyflie_hardware_gateway_interfaces::srv::AddCrazyflie::Response> response)
  {
    auto [success, msg] = add_crazyflie(request->id, request->channel, request->initial_position, request->type);
    response->success = success;
    response->msg = msg;
  }

  void handle_remove_crazyflie(const std::shared_ptr<crazyflie_hardware_gateway_interfaces::srv::RemoveCrazyflie::Request> request,
                               std::shared_ptr<crazyflie_hardware_gateway_interfaces::srv::RemoveCrazyflie::Response> response)
  {
    auto [success, msg] = remove_crazyflie(request->id, request->channel);

    response->success = success;
    response->msg = msg;
  }

private: 
  rclcpp::NodeOptions
  create_node_options(int id, 
                      int channel, 
                      const geometry_msgs::msg::Point & initial_position,
                      const std::string & type)
  {


    std::vector<std::string> remap_rules;

    remap_rules.push_back("--ros-args");
    remap_rules.push_back("-r");
    remap_rules.push_back("__node:=cf" + std::to_string(id));

    auto add_parameter = [&](const std::string &name, const std::string &value) {
      remap_rules.push_back("-p");
      remap_rules.push_back(name + ":=" + value);
    };

    add_parameter("id", std::to_string(id));
    add_parameter("channel", std::to_string(channel));
    add_parameter("datarate", "2");

    std::ostringstream pos_stream;
    pos_stream << std::fixed << std::setprecision(1) << "[" 
              << initial_position.x << "," 
              << initial_position.y << "," 
              << initial_position.z << "]";
    
    add_parameter("initial_position", pos_stream.str());

    add_parameter("send_external_position", get_parameter("crazyflieTypes." +  type + ".sendExternalPosition").as_bool() ? "True" : "False");
    add_parameter("send_external_pose", get_parameter("crazyflieTypes." +  type + ".sendExternalPose").as_bool() ? "True" : "False");
    
    add_parameter("max_initial_deviation", std::to_string(get_parameter("crazyflieTypes." +  type + ".maxInitialDeviation").as_double()));
    add_parameter("marker_configuration_index", std::to_string(get_parameter("crazyflieTypes." +  type + ".markerConfigurationIndex").as_int()));
    add_parameter("dynamics_configuration_index",std::to_string(get_parameter("crazyflieTypes." +  type + ".dynamicsConfigurationIndex").as_int()));
    
    remap_rules.push_back("--params-file");
    remap_rules.push_back(get_parameter("crazyflie_configuration_yaml").as_string());

    auto options = rclcpp::NodeOptions()
      .arguments(remap_rules);
      return options;
  }

  std::vector<std::pair<std::string, std::string>>
  get_component_resources(
    const std::string & package_name, const std::string & resource_index) const
  {
    std::string content;
    std::string base_path;
    if (
      !ament_index_cpp::get_resource(
        resource_index, package_name, content, &base_path))
    {
      throw GatewayException("Could not find requested resource in ament index");
    }

    std::vector<std::pair<std::string, std::string>> resources;
    std::vector<std::string> lines = rcpputils::split(content, '\n', true);
    for (const auto & line : lines) {
      std::vector<std::string> parts = rcpputils::split(line, ';');
      if (parts.size() != 2) {
        throw GatewayException("Invalid resource entry");
      }

      std::filesystem::path library_path = parts[1];
      if (!library_path.is_absolute()) {
        library_path = (base_path / library_path);
      }
      resources.push_back({parts[0], library_path.string()});
    }
    return resources;
  }

  std::shared_ptr<rclcpp_components::NodeFactory>
  create_component_factory(const std::string & package_name, const std::string & class_name)
  {
    auto resources = get_component_resources(package_name, "rclcpp_components");
    
    std::string library_path = resources[0].second;
    std::string fq_class_name = "rclcpp_components::NodeFactoryTemplate<" + class_name + ">";

    class_loader::ClassLoader * loader;
    RCLCPP_INFO(get_logger(), "Load Library: %s", library_path.c_str());
    try {
      loader_ = std::make_unique<class_loader::ClassLoader>(library_path);
    } catch (const std::exception & ex) {
      throw GatewayException("Failed to load library: " + std::string(ex.what()));
    } catch (...) {
      throw GatewayException("Failed to load library");
    }
  
    loader = loader_.get();

    auto classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();
    for (const auto & clazz : classes) {
      RCLCPP_INFO(get_logger(), "Found class: %s", clazz.c_str());
      if (clazz == class_name || clazz == fq_class_name) {
        RCLCPP_INFO(get_logger(), "Instantiate class: %s", clazz.c_str());
        return loader->createInstance<rclcpp_components::NodeFactory>(clazz);
      }
    }
    return {};
  }

  rclcpp::Service<crazyflie_hardware_gateway_interfaces::srv::AddCrazyflie>::SharedPtr add_service_;
  rclcpp::Service<crazyflie_hardware_gateway_interfaces::srv::RemoveCrazyflie>::SharedPtr remove_service_;
  rclcpp::TimerBase::SharedPtr cleanup_timer_;

  std::unique_ptr<class_loader::ClassLoader> loader_;
  std::shared_ptr<rclcpp_components::NodeFactory> factory_;
  std::map<std::pair<uint8_t, uint8_t>, std::pair<rclcpp_components::NodeInstanceWrapper, DedicatedExecutorWrapper>> crazyflies_; // id, channel

protected:
  std::weak_ptr<rclcpp::Executor> executor_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto options = rclcpp::NodeOptions();
  options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<CrazyflieGateway>(exec, options);
  exec->add_node(node);
  exec->spin();
  //  while (true) 
  //  {
  //    try {
  //      exec->spin_some();
  //    } catch (const std::exception &exc)   {
  //        RCLCPP_INFO(rclcpp::get_logger("gateway"), "Caught in gateway loop: %s", exc.what());
  //    }
  //    
  //  }
  return 0;
}
