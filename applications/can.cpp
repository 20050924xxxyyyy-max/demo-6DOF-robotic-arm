#include "can.hpp"

#include "cmsis_os.h"
void dm_motor_send(sp::DM_Motor & motor, sp::FDCAN & fdcan)
{
  motor.write(fdcan.tx_data);
  fdcan.send(motor.tx_id);
}
void rm_motor_send(sp::RM_Motor & motor, sp::FDCAN & fdcan)
{
  motor.write(fdcan.tx_data);
  fdcan.send(motor.tx_id);
}

//错误检测
void motor_error_detect(sp::DM_Motor & motor, sp::FDCAN & fdcan)
{
  if ((motor.error != 1 && motor.error != 0) || !motor.is_alive(osKernelSysTick())) {
    motor.write_clear_error(fdcan.tx_data);
    fdcan.send(motor.tx_id);
    motor.write_enable(fdcan.tx_data);
    fdcan.send(motor.tx_id);
  }
}
//初始化清除错误
void motor_init_clear_error(sp::DM_Motor & motor, sp::FDCAN & fdcan)
{
  motor.write_clear_error(fdcan.tx_data);
  fdcan.send(motor.tx_id);
}
//初始化使能电机
void motor_init_enable(sp::DM_Motor & motor, sp::FDCAN & fdcan)
{
  motor.write_enable(fdcan.tx_data);
  fdcan.send(motor.tx_id);
}

void arm_init_enable()
{
  motor_init_enable(motor_j1, fdcan1);
  motor_init_enable(motor_j2, fdcan1);
  motor_init_enable(motor_j3, fdcan2);
  motor_init_enable(motor_j4, fdcan2);
  // motor_init_enable(motor_j5, fdcan2);
  //   motor_init_enable(motor_j5, fdcan1);
  // motor_init_enable(gripper::motor_gripper, fdcan3);
}

void arm_init_error_clear()
{
  motor_init_clear_error(motor_j1, fdcan1);
  motor_init_clear_error(motor_j2, fdcan1);
  motor_init_clear_error(motor_j3, fdcan2);
  motor_init_clear_error(motor_j4, fdcan2);
  // motor_init_clear_error(motor_j5, fdcan2);
  //   motor_init_clear_error(motor_j5, fdcan1);
  // motor_init_clear_error(gripper::motor_gripper, fdcan3);
}

void arm_error_detect()
{
  //   motor_error_detect(motor_j0, fdcan1);
  motor_error_detect(motor_j1, fdcan1);
  motor_error_detect(motor_j2, fdcan1);
  motor_error_detect(motor_j3, fdcan2);
  motor_error_detect(motor_j4, fdcan2);
  // motor_error_detect(motor_j5, fdcan2);
  // motor_error_detect(gripper::motor_gripper, fdcan3);
}

//can1
void send_arm_j0()
{
  motor_j0.write(fdcan1.tx_data);
  fdcan1.send(motor_j0.tx_id);
}

void send_arm_j1()
{
  motor_j1.write(fdcan1.tx_data);
  fdcan1.send(motor_j1.tx_id);
}

void send_arm_j2()
{
  motor_j2.write(fdcan1.tx_data);
  fdcan1.send(motor_j2.tx_id);
}

//can3
void send_arm_j3()
{
  motor_j3.write(fdcan2.tx_data);
  fdcan2.send(motor_j3.tx_id);
}

void send_arm_j4()
{
  motor_j4.write(fdcan2.tx_data);
  fdcan2.send(motor_j4.tx_id);
}

void send_arm_j5()
{
  motor_j5.write(fdcan2.tx_data);
  fdcan2.send(motor_j5.tx_id);
}

// void send_gripper()
// {
//   gripper::motor_gripper.write(fdcan3.tx_data);
//   fdcan3.send(gripper::motor_gripper.tx_id);
// }
