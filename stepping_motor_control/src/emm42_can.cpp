#include "stepping_motor_control/emm42_can.hpp"

namespace stepping_motor_control {


Emm42CanMotor::Emm42CanMotor(unsigned short id)
{
}

Emm42CanMotor::~Emm42CanMotor()
{
}

std::unique_ptr<can_frame> Emm42CanMotor::GetPostionControlCanFrame(short speed, 
    unsigned char acc, int pulse) const
{
    std::unique_ptr<can_frame> new_msg = std::make_unique<can_frame>();
    if(speed > 1279 || speed < -1279) return nullptr;
    if(pulse > 0xFFFFFF) return nullptr;
    new_msg->can_id = id_;
    new_msg->can_dlc = 8;
    new_msg->data[0] = 0xFD;
    new_msg->data[7] = kCheckCharacter;
    unsigned char positive_or_negative = 0x00;
    if(speed < 0)
    {
        positive_or_negative = 0x01;
        speed = -speed;
    }
    new_msg->data[1] = (positive_or_negative << 4) & 0xF0 + (speed >> 8) & 0x0F;
    new_msg->data[2] = speed & 0x00FF;
    new_msg->data[3] = acc;
    new_msg->data[4] = (pulse >> 16) & 0x000000FF;
    new_msg->data[5] = (pulse >> 8) & 0x000000FF;
    new_msg->data[6] = (pulse >> 0) & 0x000000FF;
    return std::move(new_msg);
}


}  // namespace stepping_motor_control
