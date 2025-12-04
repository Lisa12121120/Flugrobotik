#pragma once

#include <vector>
#include <string>
#include <cstring>
#include "crtp_cpp/packer/logging_packer.hpp"
#include "crtp_cpp/logic/toc_logic.hpp"     


struct LogTocEntry : public TocEntry {
    uint16_t id;
    std::string group;
    std::string name;
    LogType type;
        
    LogTocEntry(); 
    LogTocEntry(const std::vector<uint8_t>& data);
    LogTocEntry(const std::string& line);
    std::string toString() const ;

    uint8_t size() const;
    float to_float(const std::vector<uint8_t>& data) const;
    float fp16_to_float(uint16_t fp16) const;

};

void quatdecompress(uint32_t comp, float q[4]);

