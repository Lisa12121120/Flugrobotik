#include "crtp_cpp/logic/link_layer_logic.hpp"

LinkLayerLogic::LinkLayerLogic(CrtpLink* crtp_link)
    : Logic(crtp_link),
      packer(LinkLayerPacker()) {}

void LinkLayerLogic::send_nullpacket() {
    CrtpRequest request;
    request.packet = packer.nullpacket();
    link->send_packet_no_response(request);
}

void LinkLayerLogic::platform_power_down() {
    CrtpRequest request;
    request.packet = packer.platform_power_down();
    link->send_packet_no_response(request);
}

void LinkLayerLogic::reboot_to_bootloader() {
    CrtpRequest request;
    request.packet = packer.reset_init();
    link->send_packet_no_response(request);

    request.packet = packer.reset_to_bootloader();
    link->send_packet_no_response(request);
}

void LinkLayerLogic::reboot_to_firmware() {
    CrtpRequest request;
    request.packet = packer.reset_init();
    link->send_packet_no_response(request);

    request.packet = packer.reset_to_firmware();
    link->send_packet_no_response(request);
}
