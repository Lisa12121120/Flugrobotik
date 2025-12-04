#pragma once

#include "crtp_cpp/packer/crtp_packer.hpp"
#include <memory>
#include <functional>
#include <vector>
#include <cstdint>

class TocPacker : public CrtpPacker {
public:
    TocPacker(int port);

    CrtpRequest get_toc_info();
    CrtpRequest get_toc_item(uint16_t index);

protected:
    CrtpPacket prepare_packet(const std::vector<uint8_t>& data);
};
