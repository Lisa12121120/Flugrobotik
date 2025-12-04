#pragma once
#include <stdint.h>
#include <map>

#include "libcrtp/CrtpPacketQueue.hpp"
#include "libcrtp/CrtpPacket.hpp"

namespace libcrtp {

struct CrtpLinkIdentifier
{
    uint8_t channel;
    uint64_t address;
    uint8_t datarate;
    bool isBroadcast;
};

class CrtpLink
{
    public: 
        CrtpLink(
            uint8_t channel,
            uint64_t address,
            uint8_t datarate
        );

        virtual ~CrtpLink(); 

        /**
        *  Adds a to be sent out Packet to the link. 
        */
        void addPacket(CrtpPacket * packet,  CrtpResponseCallback  callback);

        /**
         * Gets a packet from the port, if available.
         * Returns true if a packet was found, false otherwise.
         * The packet is not removed from the port, call notifySuccessfullPortMessage to do so.
         */
        bool getPacket(CrtpPort port, CrtpPacket * packet);

        /**
         * Returns the port with highest priority with a packet to send inside.
         * Returns CrtpPort::NO_PORT if completely empty
         */
        CrtpPort getPriorityPort() const;
        
        /**
        *  Pass a response packet, will be crossreferenced to an yet unacknowledged request which excpected a response. 
        *  Returns False if nobody listened for a response.
        */
        bool releasePacket(CrtpPacket * packet, CrtpResponseCallback & callback);      
        
        /**
         * A nullpacket from polling was successfully sent. 
         * Reset connection stats.
        */
        void notifySuccessfullNullpacket();

        /**
         * Will remove the message from the Port because it was successfully sent out. 
        */
        void notifySuccessfullPortMessage(CrtpPort port);

        /** 
         * Notifies about a failed nullpacket attempt, returns true if link shall die.
         */
        bool notifyFailedNullpacket();

        /**
         * Notifies about a failed send attempt, returns true if link shall die
        */
        bool notifyFailedPortMessage();

        /**
         * This is called before decontrstuction.
         * This way the callbacks can be returned with false
        */
        void retrieveAllCallbacks(std::vector<CrtpResponseCallback>& callbacks);

        // Check if the link is relaxed and nullpacket can be sent
        bool isRelaxed() const;

        /**
         * Give time in ms to the link, so it can update its internal state.
         */
        void tickMs(uint8_t ms);

        /**
         * Returns the link quality as a double between 0 and 1.
         * 0 means no messages were sent, 1 means all messages were sent successfully.
         * Average over the last 64 messages.
         */
        double getLinkQuality() const;

        uint8_t getChannel() const;
        uint64_t getAddress() const;
        uint8_t getDatarate() const;
        bool isBroadcast() const;

    private: 
        void onSuccessfullMessage();
        bool onFailedMessage();
        
    private:
        std::map<CrtpPort, CrtpPacketQueue> m_crtpPortQueues;

        uint8_t m_channel;
        uint64_t m_address;
        uint8_t m_datarate; 
        bool m_isBroadcast;
    
    // Configurable parameters
    private: 
        uint8_t m_failedMessagesMaximum;
        uint32_t m_nullpacketPeriodMs;
        uint32_t m_lastSuccessfullMessageTimeoutMs;
        uint32_t m_failedMessageRetryTimeoutMs;
    
    // Internal state
    private: 
        uint8_t m_failedMessagesCount;
        uint32_t m_timeSinceLastSuccessfullMessageMs;

        uint8_t m_failedPortMessagesCount;
        uint32_t m_timeSinceLastNullpacketMs;
        uint32_t m_timeSinceLastFailedPortMessageMs;
        uint64_t m_linkQuality; // 64 bits of failed and successful messages (bits)
};

} // namespace libcrtp