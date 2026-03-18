#ifndef CAN_TASK_HPP
#define CAN_TASK_HPP

#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

inline sp::CAN can1(&hcan1);

inline sp::RM_Motor motor_j0(1, sp::RM_Motors::GM6020);
inline sp::DM_Motor motor_j1(0x01, 0x11, 3.141593, 30, 10);
inline sp::DM_Motor motor_j2(0x02, 0x12, 3.141593, 30, 10);
inline sp::DM_Motor motor_j3(0x03, 0x13, 3.141593, 30, 10);
inline sp::DM_Motor motor_j4(0x04, 0x14, 3.141593, 30, 10);
inline sp::RM_Motor motor_j5(2, sp::RM_Motors::M2006);

#endif  // CAN_TASK_HPP