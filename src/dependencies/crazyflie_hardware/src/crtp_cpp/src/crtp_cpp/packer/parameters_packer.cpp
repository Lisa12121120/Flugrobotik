
#include "crtp_cpp/packer/parameters_packer.hpp"
#include <cstring>

#define PORT_PARAMETER 0x02

#define TOC_CHANNEL 0
#define READ_CHANNEL 1
#define WRITE_CHANNEL 2
#define MISC_CHANNEL 3

ParametersPacker::ParametersPacker()
    : TocPacker(PORT_PARAMETER) {}

CrtpPacket ParametersPacker::prepare_packet(const std::vector<uint8_t>& data, uint8_t channel) {
    return CrtpPacker::prepare_packet(channel, data);
}

CrtpPacket ParametersPacker::set_parameter(uint16_t id, ParamType type, std::variant<int, double> value) {
    CrtpRequest request;
    std::vector<uint8_t> data(2); // ID (uint16_t)
    std::memcpy(&data[0], &id, 2);

    switch (type) {
        case ParamTypeUint8: {
            uint8_t val = static_cast<uint8_t>(std::get<int>(value));
            data.push_back(val);
            break;
        }
        case ParamTypeUint16: {
            uint16_t val = static_cast<uint16_t>(std::get<int>(value));
            std::vector<uint8_t> val_bytes(2);
            std::memcpy(&val_bytes[0], &val, 2);
            data.insert(data.end(), val_bytes.begin(), val_bytes.end());
            break;
        }
        case ParamTypeUint32: {
            uint32_t val = static_cast<uint32_t>(std::get<int>(value));
            std::vector<uint8_t> val_bytes(4);
            std::memcpy(&val_bytes[0], &val, 4);
            data.insert(data.end(), val_bytes.begin(), val_bytes.end());
            break;
        }
        case ParamTypeUint64: {
            uint64_t val = static_cast<uint64_t>(std::get<int>(value));
            std::vector<uint8_t> val_bytes(8);
            std::memcpy(&val_bytes[0], &val, 8);
            data.insert(data.end(), val_bytes.begin(), val_bytes.end());
            break;
        }
        case ParamTypeInt8: {
            int8_t val = static_cast<int8_t>(std::get<int>(value));
            data.push_back(static_cast<uint8_t>(val));
            break;
        }
        case ParamTypeInt16: {
            int16_t val = static_cast<int16_t>(std::get<int>(value));
            std::vector<uint8_t> val_bytes(2);
            std::memcpy(&val_bytes[0], &val, 2);
            data.insert(data.end(), val_bytes.begin(), val_bytes.end());
            break;
        }
        case ParamTypeInt32: {
            int32_t val = static_cast<int32_t>(std::get<int>(value));
            std::vector<uint8_t> val_bytes(4);
            std::memcpy(&val_bytes[0], &val, 4);
            data.insert(data.end(), val_bytes.begin(), val_bytes.end());
            break;
        }
        case ParamTypeInt64: {
            int64_t val = static_cast<int64_t>(std::get<int>(value));
            std::vector<uint8_t> val_bytes(8);
            std::memcpy(&val_bytes[0], &val, 8);
            data.insert(data.end(), val_bytes.begin(), val_bytes.end());
            break;
        }
        case ParamTypeFP16: {
            double double_val = std::get<double>(value);
            uint16_t fp16_val = double_to_fp16(double_val);
            std::vector<uint8_t> val_bytes(2);
            std::memcpy(&val_bytes[0], &fp16_val, 2);
            data.insert(data.end(), val_bytes.begin(), val_bytes.end());
            break;
        }
        case ParamTypeFloat: {
            float float_val = static_cast<float>(std::get<double>(value));
            std::vector<uint8_t> val_bytes(4);
            std::memcpy(&val_bytes[0], &float_val, 4);
            data.insert(data.end(), val_bytes.begin(), val_bytes.end());
            break;
        }
        case ParamTypeDouble: {
            double double_val = std::get<double>(value);
            std::vector<uint8_t> val_bytes(8);
            std::memcpy(&val_bytes[0], &double_val, 8);
            data.insert(data.end(), val_bytes.begin(), val_bytes.end());
            break;
        }
    }
    return prepare_packet(data, WRITE_CHANNEL);
}

uint16_t ParametersPacker::double_to_fp16(double d) {
    uint32_t f32;
    uint16_t f16;

    // Double in Float umwandeln
    float f = static_cast<float>(d);
    static_assert(sizeof(float) == sizeof(uint32_t), "float size mismatch");
    std::memcpy(&f32, &f, sizeof(f32));

    int sign = (f32 >> 31) & 0x1;
    int exponent = ((f32 >> 23) & 0xFF);
    int mantissa = f32 & 0x7FFFFF;

    if (exponent == 0xFF) { // NaN oder Infinity
        f16 = (sign << 15) | 0x7C00 | (mantissa >> 13);
    } else if (exponent == 0) { // Subnormal oder 0
        if (mantissa == 0) {
            f16 = (sign << 15);
        } else {
            // Subnormal in FP16 (nicht sehr genau)
            exponent = -112; // FP16 subnormal bias
            mantissa |= 0x800000;
            while ((mantissa & 0x800000) == 0) {
                mantissa <<= 1;
                exponent--;
            }
            mantissa &= 0x7FFFFF;
            int new_exponent = exponent + 112 + 15;
            if (new_exponent >= 31) {
                f16 = (sign << 15) | 0x7C00; // Infinity
            } else if (new_exponent <= 0) {
                f16 = (sign << 15); // Zero
            } else {
                f16 = (sign << 15) | (new_exponent << 10) | (mantissa >> 13);
            }
        }
    } else { // Normal
        int new_exponent = exponent - 127 + 15;
        if (new_exponent >= 31) {
            f16 = (sign << 15) | 0x7C00; // Infinity
        } else if (new_exponent <= 0) {
            f16 = (sign << 15); // Zero
        } else {
            f16 = (sign << 15) | (new_exponent << 10) | (mantissa >> 13);
        }
    }

    return f16;
}
