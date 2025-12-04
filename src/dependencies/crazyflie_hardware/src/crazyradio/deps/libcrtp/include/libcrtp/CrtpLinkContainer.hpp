#pragma once
#include <stdint.h>
#include <map>
#include <mutex>

#include "libcrtp/CrtpPacket.hpp"
#include "libcrtp/CrtpLink.hpp"

namespace libcrtp {

class CrtpLinkContainer
{
    public:
        CrtpLinkContainer();
        virtual ~CrtpLinkContainer();

        /**
         * Adds Link to Container, if already present will not overwrite
         */
        void addLink(uint8_t channel, uint64_t address, uint8_t datarate);

        /**
         * Removes a link from the container.
         * Returns true if the link was removed, false if it was not found.
         * The callbacks are returned to the caller, they should probably be called with a nullpacket.
         */
        bool removeLink(CrtpLinkIdentifier * link, std::vector<CrtpResponseCallback>& callbacks);

        /**
         * Returns true if the link was found and copied to the link identifier.
         * If the link is not found, false is returned.
         */
        bool getLinkIdentifier(CrtpLinkIdentifier * link, uint8_t channel, uint64_t address) const;

        /**
         * Returns true if a link with the highest priority port is found.
         * The link identifier is copied to the link parameter and the port is set to the highest priority port of the link.
         */
        bool getHighestPriorityLink(CrtpLinkIdentifier * link, CrtpPort * port) const;

        /**
         * Returns a random relaxed link which is not a broadcast link.
         * If no such link is found, false is returned.
         */
        bool getRandomRelaxedNonBroadcastLink(CrtpLinkIdentifier * link) const;

        /**
         * Get connection statistics for all links.
         */
        void getConnectionStats(std::vector<CrtpLinkIdentifier>& links, std::vector<double>& quality) const;

        /**
        * In order to call link functions with link identifiers we need these wrappers
        */
        void tickLinksMs(uint8_t ms);
        
        void linkAddPacket(CrtpLinkIdentifier * link, CrtpPacket * packet, CrtpResponseCallback callback);
        bool linkGetHighestPriorityPacket(CrtpLinkIdentifier * link, CrtpPacket * packet);
        bool linkReleasePacket(CrtpLinkIdentifier  * link_id, CrtpPacket * responsePacket, CrtpResponseCallback & callback);

        void linkNotifySuccessfullNullpacket(CrtpLinkIdentifier * link_id);
        void linkNotifySuccessfullPortMessage(CrtpLinkIdentifier * link_id, CrtpPort port);
        bool linkNotifyFailedNullpacket(CrtpLinkIdentifier * link_id);
        bool linkNotifyFailedPortMessage(CrtpLinkIdentifier * link_id);

    private: 
        void copyLinkIdentifier(CrtpLinkIdentifier * from_link, CrtpLinkIdentifier * to_link) const;
        void linkToIdentifier(const CrtpLink * link, CrtpLinkIdentifier * link_id) const;
        bool linkFromIdentifier(CrtpLink ** link, CrtpLinkIdentifier * link_id);
        std::map<std::pair<uint8_t, uint64_t>, libcrtp::CrtpLink> m_links;

        mutable std::mutex m_linksMutex;
};

} // namespace libcrtp