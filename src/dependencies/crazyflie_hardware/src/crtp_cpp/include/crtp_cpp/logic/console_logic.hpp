#pragma once

#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/logic/logic.hpp"
#include "crtp_cpp/packer/console_packer.hpp"

/**
 * @brief Logic for sending console packets.
 */
class ConsoleLogic : public Logic
{
public:
    /**
     * @brief Constructor for ConsoleLogic.
     * @param crtp_link A pointer to the CrtpLink object.
     */
    ConsoleLogic(CrtpLink *crtp_link);

    /**
     * @brief Sends a console packet.
     */
    void send_consolepacket();

private:
    virtual void crtp_response_callback(const CrtpPacket &packet) {};

private:
    ConsolePacker packer;
};
