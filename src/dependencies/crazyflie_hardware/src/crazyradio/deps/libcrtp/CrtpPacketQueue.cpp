#include "libcrtp/CrtpPacketQueue.hpp"

#include <iostream>

namespace libcrtp {

CrtpPacketQueue::CrtpPacketQueue()
    : m_out_queue()
    , m_release_queue()
{   
    
}

CrtpPacketQueue::~CrtpPacketQueue()
{

}

void CrtpPacketQueue::addPacket(
    CrtpPacket * packet,
    CrtpResponseCallback  callback
)
{
    m_out_queue.push(*packet);
    if (packet->expectsResponse)
    {
        m_release_queue.push(std::make_pair(*packet, callback));
    }
}

bool CrtpPacketQueue::getPacket(
    CrtpPacket * packet)
{
    if (m_out_queue.empty()) return false;
    *packet = m_out_queue.front();
    return true;
}

void CrtpPacketQueue::sendPacketSuccess()
{
    m_out_queue.pop();
}

bool CrtpPacketQueue::releasePacket(
    CrtpPacket * responsePacket,
    CrtpResponseCallback  & callback)
{
    // No obeys ordering functionality for now

    // Shuffles through release queue until found, if found returns true and sets callback.
    // If not found returns false, callback not set. 
    // Because of ordering we can simple append unresponded packets to back
    for (int i = 0; i < m_release_queue.size(); i++) {
        auto pair = m_release_queue.front();
        m_release_queue.pop();
    
        CrtpPacket packet = pair.first;
        callback = pair.second;
        // If packet matches response Packet 
        if (this->packetsMatch(&packet, responsePacket)) return true;
        else {
            m_out_queue.push(packet);
            m_release_queue.push(std::make_pair(packet, callback));
        }
    }

    return false;
}

void CrtpPacketQueue::retrieveAllCallbacks(std::vector<CrtpResponseCallback>& callbacks)
{
     while (!m_release_queue.empty()) {
        auto pair = m_release_queue.front();
        callbacks.push_back(pair.second);
        m_release_queue.pop();
    }

    while (!m_out_queue.empty()) {
        m_out_queue.pop();
    }
}

bool CrtpPacketQueue::isEmtpy() const
{
    return m_out_queue.empty();
}

bool CrtpPacketQueue::packetsMatch(CrtpPacket * packet, CrtpPacket * response)
{
    if (response->channel != packet->channel)  return false; 
    for (int i = 0; i < packet->matchingBytes; i++)
    {
        if (packet->data[i] != response->data[i]) return false;
    }

    return true;
}

} // namespace libcrtp