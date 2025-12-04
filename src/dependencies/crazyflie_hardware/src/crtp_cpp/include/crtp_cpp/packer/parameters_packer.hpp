#pragma once

#include "crtp_cpp/packer/toc_packer.hpp"
#include <vector>
#include <cstdint>
#include <functional>
#include <variant>

enum ParamType {
    // Format: 0bXXXXYYYY (Bit positions are described below)
    // Bits: YYYY
    // BasicTypeInformation
    // [2:0]  - Base type size (0 = 8-bit, 1 = 16-bit, 2 = 32-bit, etc.)
    // [3]    - IsNonInt (0 = Integer type, 1 = Float type)
    // [4]    - IsSigned (0 = Signed, 1 = Unsigned)
    // Bits: XXXX
    // Extended TypeInformation

    //               "LEN"   INT/FLOAT  UNSIGNED
    ParamTypeUint8  = 0x00 | (0 << 2) | (1 << 3), //  0x08,
    ParamTypeUint16 = 0x01 | (0 << 2) | (1 << 3), //  0x09,
    ParamTypeUint32 = 0x02 | (0 << 2) | (1 << 3), //  0x0A,
    ParamTypeUint64 = 0x03 | (0 << 2) | (1 << 3), //  0x0B,
    ParamTypeInt8   = 0x00 | (0 << 2) | (0 << 3), //  0x00,
    ParamTypeInt16  = 0x01 | (0 << 2) | (0 << 3), //  0x01,
    ParamTypeInt32  = 0x02 | (0 << 2) | (0 << 3), //  0x02,
    ParamTypeInt64  = 0x03 | (0 << 2) | (0 << 3), //  0x03,
    ParamTypeFP16   = 0x01 | (1 << 2) | (0 << 3), //  0x05,
    ParamTypeFloat  = 0x02 | (1 << 2) | (0 << 3), //  0x06,
    ParamTypeDouble = 0x03 | (1 << 2) | (0 << 3), //  0x07,
};

union ParamValue {
    uint8_t valueUint8;
    uint16_t valueUint16;
    uint32_t valueUint32;
    uint64_t valueUint64;
    int8_t  valueInt8;
    int16_t valueInt16;
    int32_t valueInt32;
    int64_t valueInt64;
    float valueFP16;
    float valueFloat;
    double valueDouble;
};

class ParametersPacker : public TocPacker {
public:
    ParametersPacker();

    CrtpPacket set_parameter(uint16_t id, ParamType type, std::variant<int, double> value);

protected:
    CrtpPacket prepare_packet(const std::vector<uint8_t>& data, uint8_t channel);


    uint16_t double_to_fp16(double d);
};