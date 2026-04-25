#ifndef CONTROLLERS_HPP
#define CONTROLLERS_HPP
#include "../pids.hpp"
// #include "controllers/joint_motor_controller/joint_motor_controller.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/joint/joint_controller.hpp"
#include "tools/math_tools/math_tools.hpp"

inline sp::RM_Motor motor_j0(6, sp::RM_Motors::GM6020_V);
inline sp::DM_Motor motor_j1(0x01, 0x11, 3.141593, 30, 10, true);
inline sp::DM_Motor motor_j2(0x02, 0x12, 3.141593, 30, 10, true);
inline sp::DM_Motor motor_j3(0x03, 0x13, 3.141593, 30, 10, true);
inline sp::DM_Motor motor_j4(0x04, 0x14, 3.141593, 30, 10, true);
inline sp::RM_Motor motor_j5(3, sp::RM_Motors::M2006);

constexpr float theta[6] = {sp::SP_PI, 0, 0, sp::SP_PI, sp::SP_PI / 2, 0};
constexpr float d[6] = {0, 0, -0.0415, 0.16086, 0, 0};
constexpr float a[6] = {0, 0.116, 0, 0.012, 0, 0};
constexpr float alpha[6] = {sp::SP_PI / 2, 0, -sp::SP_PI / 2, sp::SP_PI / 2, -sp::SP_PI / 2, 0};

constexpr float MAX_J0 = 10000;     //rad
constexpr float MAX_J1 = 3.3;     //rad
constexpr float MAX_J2 = 1.7;     //rad
constexpr float MAX_J3 = 3;     //rad
constexpr float MAX_J4 = 0.35;     //rad
constexpr float MAX_J5 = 10000.00;  //rad

// pi/2 -5
constexpr float MIN_J0 = -10000;  //rad
constexpr float MIN_J1 = -0.3;  //rad
constexpr float MIN_J2 = -3.3;  //rad
constexpr float MIN_J3 = -3;  //rad
constexpr float MIN_J4 = -3;  //rad
constexpr float MIN_J5 = -10000;  //rad

constexpr float MID_J0 =  0;    //rad 调试3.16 操作手6.3
constexpr float MID_J1 =  -1.69;  //rad
constexpr float MID_J2 =  3;  //rad
constexpr float MID_J3 =  2.36;  //rad
constexpr float MID_J4 =  -1.8;    //rad
constexpr float MID_J5 =  0;         //rad

inline JointMotorController<sp::RM_Motor> arm_j0(
  MID_J0, -10000, 10000, MIN_J0, MAX_J0, 100, false, motor_j0, j0_pos_pid, j0_speed_pid, true);
inline JointMotorController<sp::DM_Motor> arm_j1(
  MID_J1, -10000, 10000, MIN_J1, MAX_J1, 100, false, motor_j1, j1_pos_pid, j1_speed_pid, true);
inline JointMotorController<sp::DM_Motor> arm_j2(
  MID_J2, -10000, 10000, MIN_J2, MAX_J2, 100, true, motor_j2, j2_pos_pid, j2_speed_pid, true);
inline JointMotorController<sp::DM_Motor> arm_j3(
  MID_J3, -10000, 10000, MIN_J3, MAX_J3, 100, false, motor_j3, j3_pos_pid, j3_speed_pid, true);
inline JointMotorController<sp::DM_Motor> arm_j4(
  MID_J4, -10000, 10000, MIN_J4, MAX_J4, 100, false, motor_j4, j4_pos_pid, j4_speed_pid, true);
inline JointMotorController<sp::RM_Motor> arm_j5(
  MID_J5, -10000, 10000, MIN_J5, MAX_J5, 100, false, motor_j5, j5_pos_pid, j5_speed_pid, true);

// inline JointController arm_joints(6, 0.01f, 20, 0.001, MAX_J0_T, 0.5);
#endif  // CONTROLLERS_HPP