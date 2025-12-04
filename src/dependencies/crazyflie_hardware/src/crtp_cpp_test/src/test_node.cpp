#include <cstdio>
#include "rclcpp/rclcpp.hpp"

#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/packer/toc_packer.hpp"

#include "crtp_cpp/logic/toc_logic.hpp"


#include "crtp_cpp/logic/generic_commander_logic.hpp"
#include "crtp_cpp/logic/hl_commander_logic.hpp"
#include "crtp_cpp/logic/console_logic.hpp"
//#include "crtp_cpp/logic/hl_commander_logic.hpp"
//#include "crtp_cpp/logic/link_layer_logic.hpp"


class Link : public CrtpLink
{
public:
  Link(int channel, std::array<uint8_t, 5> address, int datarate) : CrtpLink(channel, address, datarate)
  {
  }

  void send_packet_no_response(CrtpRequest request) override
  {
    printf("%d; %d", request.packet.channel, request.packet.port) ;
  }

  std::optional<CrtpPacket> send_packet(CrtpRequest request)override
  {
    return CrtpPacket();

  }
  std::vector<CrtpPacket> send_batch_request(const std::vector<CrtpRequest>)override
  {
    std::vector<CrtpPacket> vec;
    return vec;
  }
 

};



int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  Link link(10, {1, 2,3, 4, 5}, 250);
  HighLevelCommanderLogic logic(&link);
  logic.send_stop(5);

  printf("hello world crtp_cpp_test package\n");
  return 0;
}
