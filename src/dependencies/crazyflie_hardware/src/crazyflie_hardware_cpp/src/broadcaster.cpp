#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/callback_group.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "crazyflie_interfaces/msg/pose_stamped_array.hpp"
#include "broadcaster_interfaces/srv/posi_pose_broadcast_object.hpp"
#include "crtp_cpp/packer/crtp_packer.hpp"


#include "crazyflie_hardware_cpp/crtp_link_ros.hpp"

#include <map>
#include <string>

#define PORT_LOCALIZATION 6
#define POSITION_CHANNEL 2

class Broadcaster : public rclcpp_lifecycle::LifecycleNode {
public:
    Broadcaster(const rclcpp::NodeOptions &options) 
    : rclcpp_lifecycle::LifecycleNode("broadcaster", options)
    , m_decay_time(1.0)
    , m_packer(PORT_LOCALIZATION)
    , m_positions()
    , m_objects()
    , m_links()
    {
        this->declare_parameter("world", "world");
        m_world = this->get_parameter("world").as_string();


        m_add_remove_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        m_add_object_service = this->create_service<broadcaster_interfaces::srv::PosiPoseBroadcastObject>(
            "add_posi_pose_object",
            std::bind(&Broadcaster::add_object_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            m_add_remove_callback_group);
        m_remove_object_service = this->create_service<broadcaster_interfaces::srv::PosiPoseBroadcastObject>(
            "remove_posi_pose_object",
            std::bind(&Broadcaster::add_object_callback, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            m_add_remove_callback_group);



        m_callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(100)).best_effort().durability_volatile();
        auto sub_opt = rclcpp::SubscriptionOptions();
        sub_opt.callback_group = m_callback_group;
        m_position_subscription = this->create_subscription<crazyflie_interfaces::msg::PoseStampedArray>(
            "/cf_positions", 
            qos,
            std::bind(&Broadcaster::position_callback, this, std::placeholders::_1), 
            sub_opt);
    }

    

private:
    void run() {
        for (const auto &[link_key, link] : m_links) {
            if (!link->initialized) continue;
            std::map<uint8_t, std::vector<int16_t>> link_objects;
            for (const auto &[frame, object_link_key] : m_objects) {
                if (link_key != object_link_key) continue;

                auto pose_opt = get_position(frame);
                if (!pose_opt) continue;
                auto pose = pose_opt.value();

                uint8_t id = get_id_from_frame(frame);
                std::vector<int16_t> position = {
                    (int16_t)(pose.pose.position.x * 1000),
                    (int16_t)(pose.pose.position.y * 1000),
                    (int16_t)(pose.pose.position.z * 1000)};
                link_objects[id] = position;
            }
            if (!link_objects.empty()) {
                send_external_positions(link, link_objects); // Call here
            }
        }
    }

    void send_external_positions(const std::shared_ptr<RosLink>& link, 
        const std::map<uint8_t, std::vector<int16_t>>& link_objects) {
        if (!link) {
            RCLCPP_WARN(this->get_logger(), "Invalid RosLink object, skipping transmission.");
            return;
        }
        std::vector<uint8_t> data;
        size_t count = 0;
        for (const auto &[id, position] : link_objects)
        {
            data.push_back(id);
            for (uint16_t value : position) {
                data.push_back(static_cast<uint8_t>(value & 0xFF));  // Low byte
                data.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));  // High byte
            }
            count++;
            if (count == 4) {
                CrtpRequest request;
                request.packet = m_packer.prepare_packet(POSITION_CHANNEL, data);
                link->send_packet_no_response(request);
                data.clear();
                count = 0;
            }
        }
    
        // If there are remaining objects that didn't fill a full packet, send them
        if (!data.empty())
        {
            CrtpRequest request;
            request.packet = m_packer.prepare_packet(POSITION_CHANNEL, data); 

            link->send_packet_no_response(request);
        } 
    }

    void position_callback(const crazyflie_interfaces::msg::PoseStampedArray::SharedPtr msg) {
        for (const auto &pose : msg->poses) {
            m_positions[pose.header.frame_id] = pose;       
        }
        run();
    }

    std::optional<geometry_msgs::msg::PoseStamped> get_position(const std::string& frame) 
    {
        // Retrieve a position. But if it is to old -> reject.
        std::unique_lock<std::mutex> lock(m_mutex);
        auto it = m_positions.find(frame);
        if (it != m_positions.end()) {
            geometry_msgs::msg::PoseStamped pose = it->second;
            rclcpp::Time pose_time = rclcpp::Time(pose.header.stamp);

            rclcpp::Time now = this->get_clock()->now();
            rclcpp::Duration decay_duration = rclcpp::Duration::from_seconds(m_decay_time);
            rclcpp::Time decay_time = now - decay_duration;
            if (pose_time < decay_time) {
                m_positions.erase(it);
                return std::nullopt;
            }

            return pose;
        }
        return std::nullopt;
    }

    int get_id_from_frame(const std::string &frame) {
        size_t pos = 0;
        while (pos < frame.length() && !isdigit(frame[pos])) ++pos;
        return (pos < frame.length()) ? std::stoi(frame.substr(pos)) : 0;
    }

    void add_object_callback(const std::shared_ptr<broadcaster_interfaces::srv::PosiPoseBroadcastObject::Request> request,
        std::shared_ptr<broadcaster_interfaces::srv::PosiPoseBroadcastObject::Response> response) {
            response->success = true;
            
            std::unique_lock<std::mutex> lock(m_mutex, std::defer_lock);
            std::pair<int, uint8_t> channel_datarate = {request->channel, request->data_rate};
            
            lock.lock();
            m_objects[request->tf_frame_id] = channel_datarate;

            auto it = m_links.find(channel_datarate);
            if (it != m_links.end()) 
            { // Already in list.  
                if (!it->second->initialized) it->second->try_initialize(this->shared_from_this());
            } else {
                lock.unlock();
                std::array<uint8_t, 5> address = {0xFF, 0xE7, 0xE7, 0xE7, 0xE7}; // Fixed for Broadcasts
                // The link creation might take up time because it is waited for the radio.
                auto link = std::make_shared<RosLink>(this->shared_from_this(), request->channel, address, request->data_rate);
                lock.lock();
                m_links[channel_datarate] = link;
            } // lock is locked after this !!
            
            if (!m_links[channel_datarate]->initialized)
            {
                response->success = false;        
                std::string tf_frame_id_str(request->tf_frame_id.begin(), request->tf_frame_id.end());
                RCLCPP_WARN(this->get_logger(), "Adding %s to broadcaster failed. Radio not available.", tf_frame_id_str.c_str());
            }
    }

    void remove_object_callback(const std::shared_ptr<broadcaster_interfaces::srv::PosiPoseBroadcastObject::Request> request,
        std::shared_ptr<broadcaster_interfaces::srv::PosiPoseBroadcastObject::Response> response) {
            std::unique_lock<std::mutex> lock(m_mutex);

            auto it = m_objects.find(request->tf_frame_id);
            if (it != m_objects.end()) {
                m_objects.erase(it);
                response->success = true;
            } else {
                response->success = false;
            }
    }



    
private: 
    double m_decay_time;
    CrtpPacker m_packer;
    std::string m_world;
        
    rclcpp::CallbackGroup::SharedPtr m_add_remove_callback_group;
    rclcpp::Service<broadcaster_interfaces::srv::PosiPoseBroadcastObject>::SharedPtr m_add_object_service;
    rclcpp::Service<broadcaster_interfaces::srv::PosiPoseBroadcastObject>::SharedPtr m_remove_object_service;

    rclcpp::CallbackGroup::SharedPtr m_callback_group;
    rclcpp::Subscription<crazyflie_interfaces::msg::PoseStampedArray>::SharedPtr  m_position_subscription;
    rclcpp::TimerBase::SharedPtr m_broadcast_timer;


    mutable std::mutex m_mutex;
    std::map<std::string, geometry_msgs::msg::PoseStamped> m_positions;
    std::map<std::string, std::pair<int, int>> m_objects; // Pair of channel, datarate
    std::map<std::pair<int, int>, std::shared_ptr<RosLink>> m_links;
};


int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<Broadcaster>(options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
