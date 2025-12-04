#include "rclcpp/rclcpp.hpp"
#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_interfaces/srv/crtp_packet_send.hpp"

#include "crtp_interfaces/msg/crtp_link.hpp"
#include "crtp_interfaces/msg/crtp_response.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

class RosLink : public CrtpLink
{
public:

    RosLink(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node, int channel, std::array<uint8_t, 5> address, int datarate);

    bool try_initialize(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node);

    void close_link() override;

    void add_callback(uint8_t port, const CrtpCallbackType& callback) override;

    void send_packet_no_response(CrtpRequest request) override;

    std::optional<CrtpPacket> send_packet(CrtpRequest request) override;
    
    std::vector<CrtpPacket> send_batch_request(const std::vector<CrtpRequest> requests) override;

    bool initialized;
private: 
    void fill_crtp_request(std::shared_ptr<crtp_interfaces::srv::CrtpPacketSend::Request> req, const CrtpRequest& request);
    CrtpPacket response_to_packet(std::shared_ptr<crtp_interfaces::srv::CrtpPacketSend::Response> response);
    CrtpPacket ros_packet_to_packet(const crtp_interfaces::msg::CrtpPacket& ros_packet);

    void crtp_link_end_callback(const crtp_interfaces::msg::CrtpLink::SharedPtr msg);
    void crtp_response_callback(const crtp_interfaces::msg::CrtpResponse::SharedPtr msg);



private:
    std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node;
    std::string logger_name;
    
    rclcpp::CallbackGroup::SharedPtr callback_group; 
    rclcpp::Client<crtp_interfaces::srv::CrtpPacketSend>::SharedPtr send_crtp_packet_client;

    rclcpp::Subscription<crtp_interfaces::msg::CrtpLink>::SharedPtr link_end_sub;
    rclcpp::Subscription<crtp_interfaces::msg::CrtpResponse>::SharedPtr crtp_response_sub;

    rclcpp::Publisher<crtp_interfaces::msg::CrtpLink>::SharedPtr link_close_pub;

    std::map<uint8_t, std::vector<CrtpCallbackType>> callbacks;
};