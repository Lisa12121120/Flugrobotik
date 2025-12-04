#pragma once

#include <stdint.h>
#include <queue>
#include <functional>

#include "CrtpPacket.hpp"

namespace libcrtp {


class CrtpPacketQueue
{

    public: 
        CrtpPacketQueue();

        virtual ~CrtpPacketQueue();

        void addPacket(CrtpPacket * packet, CrtpResponseCallback  callback);

        void sendPacketSuccess();
        
        bool getPacket(CrtpPacket * packet);

        bool releasePacket(CrtpPacket * packet, CrtpResponseCallback &  callback);
        
        void retrieveAllCallbacks(std::vector<CrtpResponseCallback>& callbacks);

        bool isEmtpy() const;

    private: 
        bool packetsMatch(CrtpPacket * packet, CrtpPacket * response);

        std::queue<CrtpPacket> m_out_queue;
        std::queue<std::pair<CrtpPacket, CrtpResponseCallback>> m_release_queue;

};

} // namespace libcrtp