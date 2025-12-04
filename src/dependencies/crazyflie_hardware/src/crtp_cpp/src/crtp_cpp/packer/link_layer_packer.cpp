
#include "crtp_cpp/packer/link_layer_packer.hpp"
#include <cstring>

#define PORT_LINK 0xF

#define CHANNEL_ECHO 0
#define CHANNEL_SOURCE 1
#define CHANNEL_SINK 2
#define CHANNEL_NULL 3

#define CHANNEL_LINK 0xF

#define BOOTLOADER_HEADER 0xFE
#define BOOTLOADER_CMD_ALLOFF 0x01
#define BOOTLOADER_CMD_SYSOFF 0x02
#define BOOTLOADER_CMD_SYSON 0x03
#define BOOTLOADER_CMD_GETVBAT 0x04
#define BOOTLOADER_CMD_RESET_INIT 0xFF
#define BOOTLOADER_CMD_RESET 0xF0

#define BOOTLOADER_RESET_TO_BOOTLOADER 0
#define BOOTLOADER_RESET_TO_FIRMWARE 1

LinkLayerPacker::LinkLayerPacker()
    : CrtpPacker(PORT_LINK) {}

CrtpPacket LinkLayerPacker::prepare_packet(uint8_t channel, const std::vector<uint8_t>& data) {
    return CrtpPacker::prepare_packet(channel, data);
}

CrtpPacket LinkLayerPacker::echopacket() {
    std::vector<uint8_t> data;
    return prepare_packet(CHANNEL_ECHO, data);
}

CrtpPacket LinkLayerPacker::sourcepacket() {
    std::vector<uint8_t> data;
    return prepare_packet(CHANNEL_SOURCE, data);
}

CrtpPacket LinkLayerPacker::sinkpacket() {
    std::vector<uint8_t> data;
    return prepare_packet(CHANNEL_SINK, data);
}

CrtpPacket LinkLayerPacker::nullpacket() {
    std::vector<uint8_t> data;
    return prepare_packet(CHANNEL_NULL, data);
}

CrtpPacket LinkLayerPacker::get_vbat() {
    std::vector<uint8_t> data = {BOOTLOADER_HEADER, BOOTLOADER_CMD_GETVBAT};
    return prepare_packet(CHANNEL_LINK, data);
}

CrtpPacket LinkLayerPacker::platform_power_down() {
    std::vector<uint8_t> data = {BOOTLOADER_HEADER, BOOTLOADER_CMD_ALLOFF};
    return prepare_packet(CHANNEL_LINK, data);
}

CrtpPacket LinkLayerPacker::stm_power_down() {
    std::vector<uint8_t> data = {BOOTLOADER_HEADER, BOOTLOADER_CMD_SYSOFF};
    return prepare_packet(CHANNEL_LINK, data);
}

CrtpPacket LinkLayerPacker::stm_power_up() {
    std::vector<uint8_t> data = {BOOTLOADER_HEADER, BOOTLOADER_CMD_SYSON};
    return prepare_packet(CHANNEL_LINK, data);
}

CrtpPacket LinkLayerPacker::reset_init() {
    std::vector<uint8_t> data = {BOOTLOADER_HEADER, BOOTLOADER_CMD_RESET_INIT};
    return prepare_packet(CHANNEL_LINK, data);
}

CrtpPacket LinkLayerPacker::reset_to_bootloader() {
    std::vector<uint8_t> data = {BOOTLOADER_HEADER, BOOTLOADER_CMD_RESET, BOOTLOADER_RESET_TO_BOOTLOADER};
    return prepare_packet(CHANNEL_LINK, data);
}

CrtpPacket LinkLayerPacker::reset_to_firmware() {
    std::vector<uint8_t> data = {BOOTLOADER_HEADER, BOOTLOADER_CMD_RESET, BOOTLOADER_RESET_TO_FIRMWARE};
    return prepare_packet(CHANNEL_LINK, data);
}