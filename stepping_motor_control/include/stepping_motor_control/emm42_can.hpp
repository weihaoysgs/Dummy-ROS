#ifndef __EMM42_CAN_HPP_
#define __EMM42_CAN_HPP_

#include <linux/can.h>
#include <memory>

namespace stepping_motor_control {


class Emm42CanMotor
{
public:
    Emm42CanMotor(unsigned short id);
    ~Emm42CanMotor();
    std::unique_ptr<can_frame> GetPostionControlCanFrame(short speed, 
        unsigned char acc, int pulse) const;
    unsigned short get_id() {return id_;}
private:
    unsigned short id_;
    const unsigned char kCheckCharacter = 0x6B;
};


}  // namespace stepping_motor_control

#endif  // __EMM42_CAN_HPP_