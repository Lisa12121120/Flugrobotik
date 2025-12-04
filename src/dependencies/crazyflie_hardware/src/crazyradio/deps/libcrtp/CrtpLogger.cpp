#include "libcrtp/CrtpLogger.hpp"

namespace libcrtp {


CrtpLogger::CrtpLogger(bool logEnabled, 
                       const std::string& logFileName)
    : m_logEnabled(logEnabled)
{
    if (m_logEnabled) {
        m_logStream.open(logFileName);
        if (!m_logStream.is_open()) {
            throw std::runtime_error("Failed to open log file");
        }
    }
}

CrtpLogger::~CrtpLogger()
{
    if (m_logEnabled && m_logStream.is_open()) {
        m_logStream.close();
    }
}

void CrtpLogger::logCommunication(
        libcrtp::CrtpLinkIdentifier *link,
        libcrtp::CrtpPacket *packet,
        libcrtp::CrtpPacket *responsePacket,
        bool responseValid,
        std::chrono::nanoseconds logTime)
    {
        if (m_logEnabled)
        {
            std::stringstream ss;
            auto micros =  std::chrono::duration_cast<std::chrono::microseconds>(logTime).count(); //  in microseconds
            ss << "[" << (long int)(micros) << "] "; 
            ss << std::hex << (int)(uint8_t)(link->address & 0xFF);

            m_logStream << ss.str() << formatCrtpPacket(packet).str() << std::endl;
            if (responseValid) m_logStream << '\t' << formatCrtpPacket(responsePacket).str();
            m_logStream << std::endl;
        }
    }

std::stringstream CrtpLogger::formatCrtpPacket(libcrtp::CrtpPacket *packet)
{
    std::stringstream ss;
    ss << std::dec << " [" << (int)packet->port << ":" << (int)packet->channel << "] ";
    for (int i = 0; i < packet->dataLength; i++) ss << std::hex << (int)packet->data[i] << " ";
    return ss;
}

} // namespace libcrtp 