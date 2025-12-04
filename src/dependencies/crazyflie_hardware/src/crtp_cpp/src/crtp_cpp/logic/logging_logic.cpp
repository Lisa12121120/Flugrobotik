#include "crtp_cpp/logic/logging_logic.hpp"
#include <tuple>
#include <iostream> // For debugging

#define PORT_LOGGING 5

LoggingLogic::LoggingLogic(
    CrtpLink * crtp_link,
    const std::string& path
) : TocLogic<LogTocEntry>(crtp_link, path, PORT_LOGGING),
    packer(LoggingPacker()) 
{
    link->add_callback(PORT_LOGGING, std::bind(&LoggingLogic::crtp_response_callback, this, std::placeholders::_1));
}

void LoggingLogic::start_block(int id, int period_ms_d10) {
    link->send_packet(packer.start_block(id, period_ms_d10));
}

void LoggingLogic::stop_block(int id) {
    link->send_packet(packer.stop_block(id));
}

void LoggingLogic::reset() {
    link->send_packet(packer.reset());
}

void LoggingLogic::add_block(int id, const std::vector<std::string>& variables) {
    std::vector<std::pair<uint8_t, uint16_t>> vars;
    blocks[id] = std::vector<LogTocEntry>();

    for (const auto& variable_name : variables) {
        size_t dot = variable_name.find('.');

        if (dot != std::string::npos) {
            std::string group = variable_name.substr(0, dot);
            std::string name = variable_name.substr(dot + 1);

            for (const auto& entry : LoggingLogic::toc_entries) {
                if (entry.group == group && entry.name == name) {
                    vars.push_back({entry.type, entry.id});
                    blocks[id].push_back(entry);
                }
            } // else "Error: Element not found for variable"
        }
    }    
    
    link->send_packet(packer.create_block(id, vars));
}

std::vector<float> LoggingLogic::unpack_block(int block_id, const std::vector<uint8_t>& data) {
    if (blocks.find(block_id) == blocks.end()) { // Use find() for map lookup
        return {};
    }
    auto variables = blocks.at(block_id); // Use at() for safe access
    std::vector<float> unpacked_data;

    // Unpack data using struct-like functionality
    uint8_t offset = 0;
    for (const auto& variable : variables) {
        uint8_t entry_size = variable.size();
        if (data.size() < offset + entry_size) {
            return {}; // "Error: Not enough data to unpack block" 
        }
        std::vector<uint8_t> entry_data(data.begin() + offset, data.begin() + offset + entry_size);
        unpacked_data.push_back(variable.to_float(entry_data));
        offset += entry_size;
    }
    return unpacked_data;
}