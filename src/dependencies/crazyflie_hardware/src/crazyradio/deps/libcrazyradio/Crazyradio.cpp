#include "libcrazyradio/Crazyradio.hpp"


#include <sstream>
#include <iostream>
#include <stdexcept>
#include <cstring>

#include <libusb-1.0/libusb.h>
namespace libcrazyradio {
enum
{
    SET_RADIO_CHANNEL   = 0x01,
    SET_RADIO_ADDRESS   = 0x02,
    SET_DATA_RATE       = 0x03,
    SET_RADIO_POWER     = 0x04,
    SET_RADIO_ARD       = 0x05,
    SET_RADIO_ARC       = 0x06,
    ACK_ENABLE          = 0x10,
    SET_CONT_CARRIER    = 0x20,
    SCANN_CHANNELS      = 0x21,
    START_STOP          = 0x23,
    LAUNCH_BOOTLOADER   = 0xFF,
};

Crazyradio::Crazyradio() 
    : USBDevice(0x1915, 0x7777),
    m_datarate(Datarate_250KPS),
    m_address(0xDEADBEEF),
    m_channel(0),
    m_ackEnable(false) // need to be differnt in order to initially set
{
    bool success = false;
    std::vector<std::string> errors; // Store errors for potential later use.

    for (int deviceId = 0; deviceId < 5; deviceId++) {
        try {
            open(deviceId);
            success = true; // Open succeeded, so break the loop
            break;
        } catch (const std::runtime_error& e) {
            errors.push_back(e.what()); // Store the error message
        }
    }
    if (!success) {
        //If open did not succeed, throw an error.
        std::string combinedError = "Failed to open any device: ";
        for(const auto& error : errors) {
            combinedError += error + "; ";
        }
        throw std::runtime_error(combinedError);
    }
    setDatarate(Datarate_2MPS);
    setChannel(2);
    setContCarrier(false);
    setAddress(0xE7E7E7E7E7);
    setPower(Power_0DBM);
    setArc(3);
    setArdBytes(32);
    setAckEnable(true);

    std::cerr << "Crazyradio USB starting; ";
    #ifdef LEGACY_RADIO
        std::cerr << "LEGACY_RADIO: ON" << std::endl;
    #else
        std::cerr << "LEGACY_RADIO: OFF" << std::endl;
        sendVendorSetup(START_STOP, 1, 0, NULL, 0); // Send a start command to the radio.
    #endif
}

Crazyradio::~Crazyradio()
{   
    #ifndef LEGACY_RADIO
        sendVendorSetup(START_STOP, 0, 0, NULL, 0); // Send a stop command to the radio.
    #endif
    std::cerr << "Crazyradio USB stopped." << std::endl;
}



bool Crazyradio::sendCrtpPacket(
        libcrtp::CrtpLinkIdentifier * link,
        libcrtp::CrtpPacket * packet,
        libcrtp::CrtpPacket * responsePacket)
{   
    setToCrtpLink(link);
    libcrazyradio::Crazyradio::Ack ack;

    #ifndef LEGACY_RADIO
        uint8_t data[5 + 32];
        data[4] = (link->address >> 0) & 0xFF;
        data[3] = (link->address >> 8) & 0xFF;
        data[2] = (link->address >> 16) & 0xFF;
        data[1] = (link->address >> 24) & 0xFF;
        data[0] = (link->address >> 32) & 0xFF;
    
        data[5] = packet->port << 4 | packet->channel;
        memcpy(&data[6], &packet->data, packet->dataLength);
        sendPacket(data, 5 + 1 + packet->dataLength, ack);
    #else
        uint8_t data[32];
        data[0] = packet->port << 4 | packet->channel;
        memcpy(&data[1], &packet->data, packet->dataLength);
        sendPacket(data, 1 + packet->dataLength, ack);
    #endif

    
    if (link->isBroadcast) return true;    
    if (!ack.ack) {
        return false;
    } else if (!ack.size)
    {
        /* The Bug in https://github.com/bitcraze/crazyflie-firmware/issues/703 prevents a response from beeing sent back from the crazyflie.
            *  The message however gets succesfully received by the crazyflie.
            *  For now we just assume that a nullpacket would have been sent from crazyflie, in order not to break any other code.
            */
        std::cerr <<  "Empty response #703" << std::endl;
        memcpy(responsePacket, &libcrtp::nullPacket, sizeof(libcrtp::CrtpPacket));
        return true;
    } 
    
    ackToCrtpPacket(&ack, responsePacket);
    return true;
}

void Crazyradio::setToCrtpLink(libcrtp::CrtpLinkIdentifier * link)
{
    setChannel(link->channel);
    setAddress(link->address);
    setAckEnable(! link->isBroadcast); 
   
    switch (link->datarate) 
    {
        case 2: 
            setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_2MPS);
            break;
        case 1: 
            setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_1MPS);
            break;
        default: 
            setDatarate(libcrazyradio::Crazyradio::Datarate::Datarate_250KPS);
    }  
}

void Crazyradio::setChannel(uint8_t channel)
{
    if (m_channel != channel)
    {
        sendVendorSetup(SET_RADIO_CHANNEL, channel, 0, NULL, 0);
        m_channel = channel;
    }
}

void Crazyradio::setAddress(uint64_t address)
{
    #ifndef LEGACY_RADIO
        m_address = address;
        return; // Now done via the packet to the radio.
    #endif
    
    if (m_address != address) {
        unsigned char a[5];
        a[4] = (address >> 0) & 0xFF;
        a[3] = (address >> 8) & 0xFF;
        a[2] = (address >> 16) & 0xFF;
        a[1] = (address >> 24) & 0xFF;
        a[0] = (address >> 32) & 0xFF;

        //sendVendorSetup(SET_RADIO_ADDRESS, 0, 0, a, 5);
        // unsigned char a[] = {0xe7, 0xe7, 0xe7, 0xe7, 0x02};

        /*int status = */libusb_control_transfer(
            m_handle,
            LIBUSB_REQUEST_TYPE_VENDOR,
            SET_RADIO_ADDRESS,
            0,
            0,
            a,
            5,
            /*timeout*/ 1000);
         //if (status != LIBUSB_SUCCESS) {
         //    std::cerr << "sendVendorSetup: " << libusb_error_name(status) << std::endl;
         //}
        m_address = address;
    }
}

void Crazyradio::setDatarate(Datarate datarate)
{
    if (m_datarate != datarate)
    {
        sendVendorSetup(SET_DATA_RATE, datarate, 0, NULL, 0);
        m_datarate = datarate;
    }
}

void Crazyradio::setPower(Power power)
{
    sendVendorSetup(SET_RADIO_POWER, power, 0, NULL, 0);
}

void Crazyradio::setArc(uint8_t arc)
{
    sendVendorSetup(SET_RADIO_ARC, arc, 0, NULL, 0);
}

void Crazyradio::setArdTime(uint16_t us)
{
    // Auto Retransmit Delay:
    // 0000 - Wait 250uS
    // 0001 - Wait 500uS
    // 0010 - Wait 750uS
    // ........
    // 1111 - Wait 4000uS

    // Round down, to value representing a multiple of 250uS
    int t = (us / 250) - 1;
    if (t < 0) {
        t = 0;
    }
    if (t > 0xF) {
        t = 0xF;
    }
    sendVendorSetup(SET_RADIO_ARD, t, 0, NULL, 0);
}

void Crazyradio::setArdBytes(uint8_t nbytes)
{
    sendVendorSetup(SET_RADIO_ARD, 0x80 | nbytes, 0, NULL, 0);
}

void Crazyradio::setAckEnable(bool enable)
{    
    #ifndef LEGACY_RADIO
        m_ackEnable = enable;
        return; // Now done via the packet to the radio.
    #endif

    if (m_ackEnable != enable) 
    {
        sendVendorSetup(ACK_ENABLE, enable, 0, NULL, 0);
        m_ackEnable = enable;
    }
}

void Crazyradio::setContCarrier(bool active)
{
    sendVendorSetup(SET_CONT_CARRIER, active, 0, NULL, 0);
}


void Crazyradio::sendPacket(
    const uint8_t * data,
    uint32_t length,
    Ack& result
)
{
    result.ack = false;
    result.size = 0;

    int status, transferred;

    if (!m_handle) throw std::runtime_error("No valid device handle!");

    status = libusb_bulk_transfer(
        m_handle, 
        (0x01 | LIBUSB_ENDPOINT_OUT),
        (uint8_t *)data,
        length, 
        &transferred,
        /*timeout*/ 100);

    if (status != LIBUSB_SUCCESS) throw std::runtime_error(libusb_error_name(status));

    if (length != (uint32_t)transferred) {
        std::stringstream sstr;
        sstr << "Did transfer " << transferred << " but " << length << " was requested!";
        throw std::runtime_error(sstr.str());
    }

    if (m_ackEnable) 
    {
        // Read result
        status = libusb_bulk_transfer(
            m_handle,
            /* endpoint*/ (0x81 | LIBUSB_ENDPOINT_IN),
            (unsigned char*)&result,
            sizeof(result) - 1,
            &transferred,
            /*timeout*/ 100);
        result.size = transferred - 1;

        if (status == LIBUSB_ERROR_TIMEOUT) 
            std::cerr << "USB readback timeout" << std::endl;
        
        if (status != LIBUSB_SUCCESS) 
            std::cerr << "USB readback failed." << std::endl;
    }
}


void Crazyradio::ackToCrtpPacket(Ack * ack, libcrtp::CrtpPacket * packet)
{
    packet->port = (libcrtp::CrtpPort)((ack->data[0] >> 4) & 0xF);
    packet->channel = ack->data[0] & 0b11;
    for (int i = 0; i < ack->size; i++) 
        packet->data[i] = ack->data[i+1];
    packet->dataLength = ack->size -1;
}




} // namepsace libcrazyradio