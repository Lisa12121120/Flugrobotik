#include "crtp_cpp/logic/logging_logic.hpp"
#include <tuple>
#include <iostream> 
#include <sstream>
#include <cstring>
#include <math.h>

#define PORT_LOGGING 5

// Constructor from data
LogTocEntry::LogTocEntry(const std::vector<uint8_t>& data)
{
    std::memcpy(&id, data.data() + 1, 2);           // Two bytes of ident
    type = (LogType)data[3];                        // One byte of type    
    group  = std::string(reinterpret_cast<const char*>(data.data() + 4)); // std::string will read unil \0 terminated 
    name = std::string(reinterpret_cast<const char*>(data.data() + 4 + group.size() + 1));
}

// Constructor from comma-separated string
LogTocEntry::LogTocEntry(const std::string& line) {
    std::istringstream lineStream(line);
    std::string token;
    // Parse ID
    std::getline(lineStream, token, ',');
    id = static_cast<uint16_t>(std::stoi(token));
    // Parse Type
    std::getline(lineStream, token, ',');
    type = (LogType)static_cast<uint8_t>(std::stoi(token));
    // Parse Group
    std::getline(lineStream, group, ',');
    // Parse Name
    std::getline(lineStream, name, ',');
}

std::string LogTocEntry::toString() const {
    std::ostringstream lineStream;
    lineStream << id << "," << type << "," << group << "," << name;
    return lineStream.str();
}

uint8_t LogTocEntry::size() const
{
  switch (type) {
    case LogTypeUint8:
    case LogTypeInt8:
        return 1;
    case LogTypeUint16:
    case LogTypeInt16:
    case LogTypeFP16:  // FP16 (Half-Precision Float) is 2 bytes
        return 2;
    case LogTypeUint32:
    case LogTypeInt32:
    case LogTypeFloat: // Standard 32-bit float
        return 4;
    default:
        return 0; // Unknown type
  }
}

float LogTocEntry::to_float(const std::vector<uint8_t>& data) const
{ 
  if (data.size() != size()) return 0.0f;

  switch (type) {
      case LogTypeUint8:
          return static_cast<float>(data[0]); // 1-byte unsigned int
      case LogTypeInt8:
          return static_cast<float>(static_cast<int8_t>(data[0])); // 1-byte signed int
      case LogTypeUint16: {
          uint16_t value;
          std::memcpy(&value, data.data(), sizeof(value));
          return static_cast<float>(value);
      }
      case LogTypeInt16: {
          int16_t value;
          std::memcpy(&value, data.data(), sizeof(value));
          return static_cast<float>(value);
      }
      case LogTypeUint32: {
          uint32_t value;
          std::memcpy(&value, data.data(), sizeof(value));
          return static_cast<float>(value);
      }
      case LogTypeInt32: {
          int32_t value;
          std::memcpy(&value, data.data(), sizeof(value));
          return static_cast<float>(value);
      }
      case LogTypeFloat: {
          float value;
          std::memcpy(&value, data.data(), sizeof(value)); // Direct float conversion
          return static_cast<float>(value);
      }
      case LogTypeFP16: {
          // FP16 to float conversion (requires additional handling)
          // Placeholder: Assuming we receive IEEE 754 half-precision in `data`
          uint16_t fp16_value;
          std::memcpy(&fp16_value, data.data(), sizeof(fp16_value));
          return fp16_to_float(fp16_value); // Convert half-precision float
      }
      default:
          return 0.0f; // Unknown type
  }
}

float LogTocEntry::fp16_to_float(uint16_t fp16) const
{
    uint32_t s = (fp16 >> 15) & 0x00000001; // sign
    uint32_t e = (fp16 >> 10) & 0x0000001f; // exponent
    uint32_t f = fp16 & 0x000003ff;         // fraction

    if (e == 0) {
        if (f == 0) {
            uint32_t result = s << 31;
            float float_result;
            std::memcpy(&float_result, &result, sizeof(float));
            return float_result;
        } else {
            while (!(f & 0x00000400)) {
                f <<= 1;
                e -= 1;
            }
            e += 1;
            f &= ~0x00000400;
        }
    } else if (e == 31) {
        if (f == 0) {
            uint32_t result = (s << 31) | 0x7f800000;
            float float_result;
            std::memcpy(&float_result, &result, sizeof(float));
            return float_result;
        } else {
            uint32_t result = (s << 31) | 0x7f800000 | (f << 13);
            float float_result;
            std::memcpy(&float_result, &result, sizeof(float));
            return float_result;
        }
    }

    e += 127 - 15;
    f <<= 13;
    uint32_t result = (s << 31) | (e << 23) | f;
    float float_result;
    std::memcpy(&float_result, &result, sizeof(float));
    return float_result;
}

void quatdecompress(uint32_t comp, float q[4])
{
	float const SMALL_MAX = 1.0 / sqrt(2);
	unsigned const mask = (1 << 9) - 1;

	int const i_largest = comp >> 30;
	float sum_squares = 0;
	for (int i = 3; i >= 0; --i) {
		if (i != i_largest) {
			unsigned mag = comp & mask;
			unsigned negbit = (comp >> 9) & 0x1;
			comp = comp >> 10;
			q[i] = SMALL_MAX * ((float)mag) / mask;
			if (negbit == 1) {
				q[i] = -q[i];
			}
			sum_squares += q[i] * q[i];
		}
	}
	q[i_largest] = sqrtf(1.0f - sum_squares);
}
