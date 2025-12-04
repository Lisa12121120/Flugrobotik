#include "libcrtp/CrtpLink.hpp"
#include <iostream>

namespace libcrtp {

CrtpLink::CrtpLink(
    uint8_t channel, 
    uint64_t address,
    uint8_t datarate)
    : m_crtpPortQueues({
        {CONSOLE,           CrtpPacketQueue()},
        {PARAMETERS,        CrtpPacketQueue()},
        {COMMANDER,         CrtpPacketQueue()},
        {MEMORY_ACCESS,     CrtpPacketQueue()},
        {DATA_LOGGING,      CrtpPacketQueue()},
        {LOCALIZATION,      CrtpPacketQueue()},
        {GENERIC_SETPOINT,  CrtpPacketQueue()},
        {PLATFORM,          CrtpPacketQueue()},
        {CLIENT_SIDE_DEBUG, CrtpPacketQueue()},
        {LINK_LAYER,        CrtpPacketQueue() }
        })
    , m_channel(channel)
    , m_address(address)
    , m_datarate(datarate)
    , m_isBroadcast(((address >> 4 * 8) & 0xFF) == 0xFF) // Broadcasting Links have 0xFF as the first byte of the address (cfs have 0xE7)   
    , m_failedMessagesMaximum(100) // After this many failed messages we consider the link dead, we also wait m_failedMessageRetryTimeout before retrying
    , m_nullpacketPeriodMs(10) // At most 100 Hz for ping messages
    , m_lastSuccessfullMessageTimeoutMs(2000) // If 2 seconds no Communication -> Fail refardless of how many messages failed before.
    , m_failedMessageRetryTimeoutMs(30) // If a message fails, we wait 30 ms before retrying
    , m_failedMessagesCount(0)
    , m_timeSinceLastSuccessfullMessageMs(0)
    , m_failedPortMessagesCount(0)
    , m_timeSinceLastNullpacketMs(0)
    , m_timeSinceLastFailedPortMessageMs(m_failedMessageRetryTimeoutMs) // Do not wait before sending first message
    , m_linkQuality(~0) // 64 bits of failed and successful messages (bits)
{
}

CrtpLink::~CrtpLink()
{
}

void CrtpLink::addPacket(
    CrtpPacket * packet,
    CrtpResponseCallback  callback)
{
    //if (packet->expectsResponse) 
    //    std::cerr << "Add: " << (int)packet->port << std::endl;
    m_crtpPortQueues[packet->port].addPacket(packet, callback);
}

bool CrtpLink::getPacket(
    CrtpPort port, 
    CrtpPacket * packet)
{   
    return m_crtpPortQueues[port].getPacket(packet);
}

bool CrtpLink::releasePacket(
    CrtpPacket * packet,
    CrtpResponseCallback &  callback)
{   
    /*
        Because log messages are not obeying the ordering process and dont have a request we cannot pass them into the queue
        otherwise other requested messages would get unvalidated
        TODO: Fix this in Crazyflie Firmware, because this does not fullfill crtp specifications as defined in:
        https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/functional-areas/crtp/
    */
    if (packet->port == CrtpPort::DATA_LOGGING && packet->channel == 2) return false;
    
    bool released = m_crtpPortQueues[packet->port].releasePacket(packet, callback);
    //if (released)
    //    std::cerr << "Released?: " << (int)packet->port << (released ? "yes" : "no") << std::endl;
    return released;
}

CrtpPort CrtpLink::getPriorityPort() const
{
    if (m_timeSinceLastFailedPortMessageMs < m_failedMessageRetryTimeoutMs)
        return CrtpPort::NO_PORT; // Wait before sending a failed packet again.

    for (const auto& [port, queue] : m_crtpPortQueues) 
    {
        if (! queue.isEmtpy()) return port; 
    }
    return CrtpPort::NO_PORT;
}

void CrtpLink::notifySuccessfullNullpacket()
{
    m_timeSinceLastNullpacketMs = 0;
    onSuccessfullMessage();
}

void CrtpLink::notifySuccessfullPortMessage(CrtpPort port)
{
    m_crtpPortQueues[port].sendPacketSuccess();
    onSuccessfullMessage();
}

bool CrtpLink::notifyFailedNullpacket()
{
    m_timeSinceLastNullpacketMs = 0;
    return onFailedMessage();
}

bool CrtpLink::notifyFailedPortMessage()
{   
    m_failedPortMessagesCount++;
    m_timeSinceLastFailedPortMessageMs = 0;
    return onFailedMessage();
}

void CrtpLink::retrieveAllCallbacks(std::vector<CrtpResponseCallback>& callbacks)
{
    for (auto& [port, queue] : m_crtpPortQueues) 
    {
        queue.retrieveAllCallbacks(callbacks); 
    }
}

void CrtpLink::tickMs(uint8_t ms)
{
    m_timeSinceLastNullpacketMs += ms;
    m_timeSinceLastSuccessfullMessageMs += ms;
    m_timeSinceLastFailedPortMessageMs += ms;
}

bool CrtpLink::isRelaxed() const
{
    // if (m_failedMessagesCount) // If we have failed messages and need to sent packets, we should not send nullpackets
    //     for (const auto& [port, queue] : m_crtpPortQueues) if (! queue.isEmtpy()) return false; 
    
    return m_timeSinceLastNullpacketMs >= m_nullpacketPeriodMs;
}


double CrtpLink::getLinkQuality() const
{
    uint64_t x = m_linkQuality;
    int count = 0;
    while (x) {
        x &= (x - 1);
        count++;
    }
    return count / 64.0;
}


uint8_t CrtpLink::getChannel() const
{
    return m_channel;
}

uint64_t CrtpLink::getAddress() const
{
    return m_address;
}

uint8_t CrtpLink::getDatarate() const
{
    return m_datarate;
}

bool CrtpLink::isBroadcast() const
{
    return m_isBroadcast;
}

void CrtpLink::onSuccessfullMessage()
{
    m_linkQuality = (m_linkQuality << 1) | 1; // shift left, add a 1 to the end of the bitfield
    
    m_failedMessagesCount = 0;
    m_failedPortMessagesCount = 0; // Also nullpackets clear this count
    m_timeSinceLastSuccessfullMessageMs = 0;
}

bool CrtpLink::onFailedMessage()
{
    m_linkQuality <<= 1; // shift left, add a 0 to the end of the bitfield

    m_failedMessagesCount++;

    /**
     * Fail after m_failedMessagesMaximum or if lastSuccessfullMessage > m_lastSuccessfullMessageTimeout
    */
    if (m_failedMessagesCount > m_failedMessagesMaximum || m_timeSinceLastSuccessfullMessageMs > m_lastSuccessfullMessageTimeoutMs) 
    {
        return true;
    }
    return false;
}

} // namespace libcrtp