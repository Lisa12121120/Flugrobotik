#include "crtp_cpp/logic/parameters_logic.hpp"
#include <stdexcept>
#include <sstream>
#include <cstring>

#define PORT_PARAMETER 0x02

// Constructor from comma-separated string
ParamTocEntry::ParamTocEntry(const std::string& csv) {
    std::istringstream lineStream(csv);
    std::string token;
    // Parse ID
    std::getline(lineStream, token, ',');
    id = static_cast<uint16_t>(std::stoi(token));
    // Parse Type
    std::getline(lineStream, token, ',');
    type = (ParamType)static_cast<uint8_t>(std::stoi(token));
    // Parse Readonly
    std::getline(lineStream, token, ',');
    readonly = token == "1";
    // Parse Group
    std::getline(lineStream, group, '.');
    // Parse Name
    std::getline(lineStream, name, ',');  
}

ParamTocEntry::ParamTocEntry(const std::vector<uint8_t>& data)
{
    std::memcpy(&id, data.data() + 1, 2);         // Two bytes of ident
    uint8_t type_info = data[3];                  // One byte of typeInfo    
    group  = std::string(reinterpret_cast<const char*>(data.data() + 4)); // std::string will read unil \0 terminated 
    name = std::string(reinterpret_cast<const char*>(data.data() + 4 + group.size() + 1));

    type = (ParamType)(type_info & 0x0F);         // 4 Bits of type (2len, int/float, unsigned/signed)
    readonly = type_info & (0x00 | (1 <<  6));    // 1 Bit if ReadOnly
}

std::string ParamTocEntry::toString() const 
{
    std::ostringstream ss;
    ss << id << "," <<  type << "," << readonly << ","  << group << "." << name;
    return ss.str();
}

bool ParamTocEntry::isInteger() const 
{
    return (type & 0x04) == 0; // If IsNonInt bit (3rd bit) is 0, it's an integer
}

bool ParamTocEntry::isDouble() const 
{
    return (type & 0x04) != 0; // If IsNonInt bit (3rd bit) is 1, it's a float
}


ParametersLogic::ParametersLogic(CrtpLink* crtp_link, const std::string& path)
    : TocLogic<ParamTocEntry>(crtp_link, path, PORT_PARAMETER),
      packer(ParametersPacker()) {}


bool ParametersLogic::send_set_parameter(const std::string& group, const std::string& name, std::variant<int, double> value) {
    for (const auto& entry : ParametersLogic::toc_entries) {
        if (entry.group == group && entry.name == name) {
            CrtpRequest request;
            request.packet = packer.set_parameter(entry.id, entry.type, value);
            link->send_packet_no_response(request);
            return true;
        }
    }
    return false;
}