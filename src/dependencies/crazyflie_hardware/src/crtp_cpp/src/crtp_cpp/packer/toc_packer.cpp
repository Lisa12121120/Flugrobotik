#include "crtp_cpp/packer/toc_packer.hpp"
#include <tuple>
#include <cstring> // For memcpy if needed

#define TOC_CHANNEL 0

// Commands used when accessing the Table of Contents
#define CMD_GET_ITEM  2  // v2; only v2 is supported
#define CMD_GET_INFO  3  // v2; only v2 is supported

TocPacker::TocPacker(int port)
    : CrtpPacker(port) {}


CrtpPacket TocPacker::prepare_packet(const std::vector<uint8_t>& data) 
{
    return CrtpPacker::prepare_packet(TOC_CHANNEL, data);
}

CrtpRequest TocPacker::get_toc_info() 
{   
    CrtpRequest request;
    std::vector<uint8_t> data = {CMD_GET_INFO};
    request.packet = prepare_packet(data);
    request.expects_response = true;
    request.matching_bytes = 1;
    return request;
}

CrtpRequest TocPacker::get_toc_item(uint16_t index) 
{   
    CrtpRequest request;
    std::vector<uint8_t> data(1 + 2); // Pre-allocate
    data[0] = CMD_GET_ITEM;
    // Pack the index using bitwise operations
    data[1] = static_cast<uint8_t>(index & 0xFF);       // Lower byte
    data[2] = static_cast<uint8_t>((index >> 8) & 0xFF); // Higher byte
    request.packet = prepare_packet(data);
    request.expects_response = true;
    request.matching_bytes = 3;
    return request;
}
