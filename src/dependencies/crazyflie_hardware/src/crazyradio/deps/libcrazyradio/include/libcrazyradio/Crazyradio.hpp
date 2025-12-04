#pragma once

#include <stdint.h>
#include "USBDevice.hpp"


#include "libcrtp/CrtpPacket.hpp"
#include "libcrtp/CrtpLink.hpp"

namespace libcrazyradio {
class Crazyradio : public USBDevice
{
public:
    struct Ack
    {
        Ack()
        : ack(0)
        , size(0)
        {}

        uint8_t ack:1;
        uint8_t powerDet:1;
        uint8_t retry:4;
        uint8_t data[32];

        uint8_t size;
    }__attribute__((packed));

    enum Datarate
    {
        Datarate_250KPS = 0,
        Datarate_1MPS   = 1,
        Datarate_2MPS   = 2,
    };

    enum Power
    {
        Power_M18DBM = 0,
        Power_M12DBM = 1,
        Power_M6DBM  = 2,
        Power_0DBM   = 3,
    };

public:
    Crazyradio();
    
    virtual ~Crazyradio();

    /**
     * Transmits a CrtpPacket over the Crazyradio.
     * Returns true if the packet was sent successfully.
     * If the link is non broadcast and the packet was sent successfully, the responsePacket will contain a response from the Crazyflie.
    */
    bool sendCrtpPacket(
        libcrtp::CrtpLinkIdentifier * link,
        libcrtp::CrtpPacket * packet,
        libcrtp::CrtpPacket * responsePacket);
        
private:
    void sendPacket(
        const uint8_t* data,
        uint32_t length, 
        Ack& result
    );

    void setToCrtpLink(libcrtp::CrtpLinkIdentifier * link);

    void setChannel(uint8_t channel);

    void setAddress(uint64_t address);

    void setDatarate(Datarate datarate);

    void setPower(Power power);

    void setArc(uint8_t arc);

    void setArdTime(uint16_t us);

    void setArdBytes(uint8_t nbytes);

    void setAckEnable(bool enable);

    void setContCarrier(bool active);

    void ackToCrtpPacket(Ack * ack, libcrtp::CrtpPacket * packet);

       
private: 
    uint8_t m_channel;
    uint64_t m_address;
    Datarate m_datarate;
    bool m_ackEnable;

};



} // namespace libcrazyradio