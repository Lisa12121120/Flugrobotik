#pragma once

#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/logic/logic.hpp"
#include "crtp_cpp/packer/toc_packer.hpp"
#include <vector>
#include <functional>
#include <tuple>
#include <optional>
#include <string>

struct TocEntry {
    public:
        //virtual TocEntry(const std::vector<uint8_t>& data);
        //virtual TocEntry(const std::string& line);
        //virtual std::string toString() const;
};

/**
 * @brief Logic for Table of Contents (TOC) related communication.
 */

template <class T>
class TocLogic : public Logic {
public:
    /**
     * @brief Constructor for TocLogic.
     * @param crtp_link A pointer to the CrtpLink object.
     * @param path Path to the TOC cache file.
     */
    TocLogic(CrtpLink* crtp_link, const std::string& path, uint8_t port);

    bool load_from_file(uint32_t crc);
    void write_to_file();
    /**
     * @brief Initializes the TOC by fetching or downloading items.
     */
    void initialize_toc();

    /**
     * @brief Sends a request to get TOC information.
     * @return A pair containing the number of items and CRC.
     */
    std::pair<uint16_t, uint32_t> send_get_toc_info();

    /**
     * @brief Sends requests to download all TOC items.
     */
    void send_download_toc_items();

protected:
    TocPacker packer;
    std::optional<uint16_t> nbr_of_items;
    std::optional<uint32_t> crc;

    std::vector<T> toc_entries;
    std::string toc_cache_path;
};