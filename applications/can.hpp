#ifndef CAN_HPP
#define CAN_HPP

#include "io/fdcan/fdcan.hpp"
#include "controllers/controllers.hpp"
inline sp::FDCAN fdcan1(&hfdcan1);
inline sp::FDCAN fdcan2(&hfdcan2);
inline sp::FDCAN fdcan3(&hfdcan3);

void arm_init_enable();
void arm_init_error_clear();
void arm_error_detect();

void send_arm_j0();
void send_arm_j1();
void send_arm_j2();
void send_arm_j3();
void send_arm_j4();
void send_arm_j5();
void send_gripper();

#endif  // CAN_HPP