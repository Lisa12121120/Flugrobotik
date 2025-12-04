#include "crtp_cpp/logic/console_logic.hpp"

ConsoleLogic::ConsoleLogic(CrtpLink *crtp_link)
    : Logic(crtp_link),
      packer(ConsolePacker())
{
    link->add_callback(PORT_CONSOLE, std::bind(&ConsoleLogic::crtp_response_callback, this, std::placeholders::_1));
}

void ConsoleLogic::send_consolepacket()
{
    CrtpRequest request;
    request.packet = packer.consolepacket();
    link->send_packet_no_response(request);
}