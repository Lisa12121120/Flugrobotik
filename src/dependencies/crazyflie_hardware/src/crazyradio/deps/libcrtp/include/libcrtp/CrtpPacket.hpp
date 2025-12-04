#pragma once

#include <stdint.h>
#include <functional>

namespace libcrtp {
    enum CrtpPort {
        CONSOLE             = 0,
        PARAMETERS          = 2,
        COMMANDER           = 3,
        MEMORY_ACCESS       = 4,
        DATA_LOGGING        = 5,
        LOCALIZATION        = 6,
        GENERIC_SETPOINT    = 7,
        PLATFORM            = 13,
        CLIENT_SIDE_DEBUG   = 14,
        LINK_LAYER          = 15,
        NO_PORT             = 0xFF
    };

    struct CrtpPacket
    {
        CrtpPort port;
        uint8_t channel;
        uint8_t data[31];
        uint8_t dataLength;
    
        bool expectsResponse;
        uint8_t matchingBytes;
        bool obeysOrdering;
    };

    inline const CrtpPacket nullPacket = {
        .port = CrtpPort::LINK_LAYER,
        .channel = 3,
        .data = {},
        .dataLength = 0,
        .expectsResponse = false,
        .matchingBytes = 0,
        .obeysOrdering = false
    };

    inline bool isNullPacket(CrtpPacket * packet) {
        return (packet->port == CrtpPort::LINK_LAYER && packet->channel == 3);
    }

    using CrtpResponseCallback = std::function<void(CrtpPacket*, bool)>;
} // namespace libcrtp