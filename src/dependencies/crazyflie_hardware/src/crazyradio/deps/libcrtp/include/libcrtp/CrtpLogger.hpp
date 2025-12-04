#pragma once

#include "libcrtp/CrtpPacket.hpp"
#include "libcrtp/CrtpLink.hpp"

#include <fstream>
#include <string.h>
#include <chrono>
#include <sstream>

namespace libcrtp {

class CrtpLogger
{
    public:
        CrtpLogger(bool logEnabled, const std::string& logFileName);

        virtual ~CrtpLogger();

        void logCommunication(
            libcrtp::CrtpLinkIdentifier *link,
            libcrtp::CrtpPacket *packet,
            libcrtp::CrtpPacket *responsePacket,
            bool responseValid,
            std::chrono::nanoseconds logTime);
    private:
        bool m_logEnabled;
        std::ofstream m_logStream;

        std::stringstream formatCrtpPacket(libcrtp::CrtpPacket *packet);
};



} // namespace libcrtp