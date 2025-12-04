#pragma once 

#include "crtp_cpp/link/crtp_link.hpp"
#include "crtp_cpp/logic/logic.hpp"
#include "crtp_cpp/packer/basic_commander_packer.hpp"

class BasicCommanderLogic : public Logic {
public: 
    BasicCommanderLogic(CrtpLink * crtp_link);
    
    void set_client_xmode(bool enable);

    void send_setpoint(float roll, float pitch, float yawrate, uint16_t thrust);

private: 
    BasicCommanderPacker packer;
    bool x_mode;    
};