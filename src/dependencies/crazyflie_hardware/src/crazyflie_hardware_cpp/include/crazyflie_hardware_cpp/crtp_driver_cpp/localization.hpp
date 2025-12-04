

#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/logic/localization_logic.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "std_msgs/msg/string.hpp"

#include "object_tracker_interfaces/srv/add_tracker_object.hpp"
#include "object_tracker_interfaces/srv/remove_tracker_object.hpp"


#include "broadcaster_interfaces/srv/posi_pose_broadcast_object.hpp"

class Localization : public LocalizationLogic {
public:
    Localization(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, CrtpLink * link, std::string tf_name);
public: 

    bool stop_external_tracking();
    bool start_external_tracking(   int marker_configuration_index,
                                    int dynamics_configuration_index, 
                                    double max_initial_deviation, 
                                    std::vector<double> initial_position,
                                    int channel,
                                    int datarate);

private: 
    bool add_to_tracker(
        int marker_configuration_index,
        int dynamics_configuration_index, 
        double max_initial_deviation,
        std::vector<double> initial_position);
    bool add_to_broadcaster(int channel, int datarate);

    bool remove_from_tracker();
    bool remove_from_broadcaster();

private: 
    std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node;
    std::string logger_name;

    bool is_beeing_tracked; 
    bool is_beeing_broadcasted;

    int channel_;
    int data_rate_;
    
    std::string tf_name;

    rclcpp::CallbackGroup::SharedPtr callback_group; 

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr console_publisher;
};