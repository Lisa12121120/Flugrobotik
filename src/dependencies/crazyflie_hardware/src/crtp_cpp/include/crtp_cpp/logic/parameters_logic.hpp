#pragma once

#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/logic/toc_logic.hpp"
#include "crtp_cpp/packer/crtp_packer.hpp"
#include "crtp_cpp/packer/parameters_packer.hpp"
#include <string>

struct ParamTocEntry : public TocEntry{
    uint16_t id;
    std::string group;
    std::string name;
    ParamType type;
    bool readonly;

    ParamTocEntry();
    /**
     * @brief Construct a ParamToc Entry from data received from crazyflie.
    */
    ParamTocEntry(const std::vector<uint8_t>& data);

    /**
     * @brief Construct a ParamTocEntry from a string. This is for saving to a file and loading again. (toString is the reverse)
    */
    ParamTocEntry(const std::string& line);
    std::string toString() const;
    bool isInteger() const;
    bool isDouble() const;
};

/**
 * @brief Logic for parameter-related communication.
 */
class ParametersLogic : public TocLogic<ParamTocEntry> {
public:
    /**
     * @brief Constructor for ParametersLogic.
     * @param crtp_link A pointer to the CrtpLink object.
     * @param path Path to the parameter TOC file.
     */
    ParametersLogic(CrtpLink* crtp_link, const std::string& path);
    
    

    /**
     * @brief Sends a packet to set a parameter.
     * @param group The parameter group.
     * @param name The parameter name.
     * @param value The parameter value.
     */
    bool send_set_parameter(const std::string& group, const std::string& name, std::variant<int, double> value);

private:
    ParametersPacker packer;
};
