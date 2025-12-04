#include "libcrtp/CrtpLinkContainer.hpp"

namespace libcrtp {

CrtpLinkContainer::CrtpLinkContainer() 
    : m_links()
{
}

CrtpLinkContainer::~CrtpLinkContainer()
{
    /* Maybe have to close links properly */
}

void CrtpLinkContainer::copyLinkIdentifier(CrtpLinkIdentifier * from_link, CrtpLinkIdentifier * to_link) const
{
    to_link->channel = from_link->channel;
    to_link->address = from_link->address;
    to_link->datarate = from_link->datarate;
    to_link->isBroadcast = from_link->isBroadcast;

}

void CrtpLinkContainer::linkToIdentifier(const CrtpLink * link, CrtpLinkIdentifier *  link_id) const
{
    link_id->channel = link->getChannel();
    link_id->address = link->getAddress();
    link_id->datarate = link->getDatarate();
    link_id->isBroadcast = link->isBroadcast();
}

bool CrtpLinkContainer::linkFromIdentifier(CrtpLink ** link, CrtpLinkIdentifier * link_id)
{
    std::pair<uint8_t, uint64_t> key = {link_id->channel,link_id->address};
    auto link_ = m_links.find(key);
    if (link_ != m_links.end()) 
    {   
        *link = &link_->second;
        return true;
    }
    return false; 
}

void CrtpLinkContainer::addLink(uint8_t channel, uint64_t address, uint8_t datarate)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink link(channel, address, datarate);
    std::pair<uint8_t, uint64_t> linkKey = {channel,address};
    m_links.insert({linkKey, link}); // If already in m_links this wont duplicate
}

bool CrtpLinkContainer::removeLink(CrtpLinkIdentifier * link_id, std::vector<CrtpResponseCallback>& callbacks)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    std::pair<uint8_t, uint64_t> linkKey = {link_id->channel,link_id->address};
    auto link = m_links.find(linkKey);
    if (link != m_links.end()) 
    {
        link->second.retrieveAllCallbacks(callbacks);
        m_links.erase(link);
        return true;
    }
    return false;
}

bool CrtpLinkContainer::getLinkIdentifier(CrtpLinkIdentifier * link, uint8_t channel, uint64_t address) const
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    std::pair<uint8_t, uint64_t> linkKey = {channel, address};
    auto link_ = m_links.find(linkKey); // A sad cpp construct
    if (link_ != m_links.end()) {
        linkToIdentifier(&link_->second , link);
        return true;
    }
    return false;
}

bool CrtpLinkContainer::getHighestPriorityLink(CrtpLinkIdentifier * link, CrtpPort * port) const
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    libcrtp::CrtpPort highestPriorityPort = libcrtp::CrtpPort::NO_PORT;
    std::pair<uint8_t, uint64_t> bestKey = {0,0};
    for (const auto& [key, link_] : m_links) 
    {       
        libcrtp::CrtpPort port = link_.getPriorityPort();
        if (link_.isBroadcast() && port != libcrtp::CrtpPort::NO_PORT) {
            // If there is a broadcast Packet. Send immediately.
            highestPriorityPort = port;
            bestKey = key;
            break;
        }
        if (port < highestPriorityPort) {
            highestPriorityPort = port;
            bestKey = key;
        } 
    }
    auto link_ = m_links.find(bestKey);
    if (highestPriorityPort != libcrtp::CrtpPort::NO_PORT && link_ != m_links.end()) 
    {   
        linkToIdentifier(&link_->second , link);
        *port = highestPriorityPort;
        return true;
    }
    return false;
}

bool CrtpLinkContainer::getRandomRelaxedNonBroadcastLink(CrtpLinkIdentifier * link) const
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    // Filter for relaxed links
    std::vector<libcrtp::CrtpLinkIdentifier> relaxed_links;

    libcrtp::CrtpLinkIdentifier link_id;
    // Iterate over the original map
    for (const auto& entry : m_links) {
        if (entry.second.isRelaxed() && !entry.second.isBroadcast()) {
            linkToIdentifier(&entry.second, &link_id);
            relaxed_links.push_back(link_id);
        }
    }

    if (relaxed_links.size())
    {
        auto it = relaxed_links.begin();
        std::advance(it, rand() % relaxed_links.size());
        copyLinkIdentifier(&(*it), link);
        return true;
    }
    return false;

}

void CrtpLinkContainer::tickLinksMs(uint8_t ms)
{

    std::unique_lock<std::mutex> mlock(m_linksMutex);
    for (auto& [key, link_] : m_links) 
    {       
        link_.tickMs(ms);
    }
}

void CrtpLinkContainer::getConnectionStats(std::vector<CrtpLinkIdentifier>& links, std::vector<double>& quality) const
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    links.clear();
    quality.clear();
    for (const auto& [key, link_] : m_links) 
    {       
        CrtpLinkIdentifier link_id;
        linkToIdentifier(&link_, &link_id);
        links.push_back(link_id);
        quality.push_back(link_.getLinkQuality());
    }
} 



void CrtpLinkContainer::linkAddPacket(CrtpLinkIdentifier * link_id, CrtpPacket * packet, CrtpResponseCallback callback)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id))
    {
        link->addPacket(packet, callback);
    }
}

bool CrtpLinkContainer::linkGetHighestPriorityPacket(CrtpLinkIdentifier * link_id, CrtpPacket * packet)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id)) {
        CrtpPort port = link->getPriorityPort();
        if (port == CrtpPort::NO_PORT) return false; // No packets available
        return link->getPacket(port, packet);
    }
    return false;
}

bool CrtpLinkContainer::linkReleasePacket(CrtpLinkIdentifier * link_id, 
                                          CrtpPacket * responsePacket, 
                                          CrtpResponseCallback & callback)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id))
    {
        return link->releasePacket(responsePacket, callback);
    }
    return false;
}

void CrtpLinkContainer::linkNotifySuccessfullNullpacket(CrtpLinkIdentifier * link_id)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id)) {
        link->notifySuccessfullNullpacket();
    }
}


void CrtpLinkContainer::linkNotifySuccessfullPortMessage(CrtpLinkIdentifier * link_id, CrtpPort port)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id)) {
        link->notifySuccessfullPortMessage(port);
    }
}



bool CrtpLinkContainer::linkNotifyFailedNullpacket(CrtpLinkIdentifier * link_id)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id))
    {
        return link->notifyFailedNullpacket();
    }
    return false;
}

bool CrtpLinkContainer::linkNotifyFailedPortMessage(CrtpLinkIdentifier * link_id)
{
    std::unique_lock<std::mutex> mlock(m_linksMutex);
    CrtpLink * link;
    if (linkFromIdentifier(&link, link_id))
    {
        return link->notifyFailedPortMessage();
    }
    return false;
}

} // namespace libcrtp