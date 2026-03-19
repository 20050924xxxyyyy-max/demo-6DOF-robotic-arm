#ifndef CONTROLLERS_HPP
#define CONTROLLERS_HPP
#include "../pids.hpp"
#include "controllers/joint_motor_controller/joint_motor_controller.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/math_tools/math_tools.hpp"

inline sp::RM_Motor motor_j0(3, sp::RM_Motors::GM6020_V);
inline sp::DM_Motor motor_j1(0x01, 0x11, 3.141593, 30, 10, true);
inline sp::DM_Motor motor_j2(0x02, 0x12, 3.141593, 30, 10, true);
inline sp::DM_Motor motor_j3(0x03, 0x13, 3.141593, 30, 10, true);
inline sp::DM_Motor motor_j4(0x04, 0x14, 3.141593, 30, 10, true);
inline sp::RM_Motor motor_j5(2, sp::RM_Motors::M2006);

constexpr float theta[6] = {0, 0, 0, sp::SP_PI, sp::SP_PI / 2, 0};
constexpr float d[6] = {0, 0, -0.10375, 0.40147, 0, 0.211};
constexpr float a[6] = {0, 0.290, 0, 0.03, 0, 0};
constexpr float alpha[6] = {sp::SP_PI / 2, 0, -sp::SP_PI / 2, sp::SP_PI / 2, -sp::SP_PI / 2, 0};

constexpr float MAX_J0 = sp::SP_PI;  //rad
constexpr float MAX_J1 = sp::SP_PI;  //rad
constexpr float MAX_J2 = sp::SP_PI;  //rad
constexpr float MAX_J3 = sp::SP_PI;  //rad
constexpr float MAX_J4 = sp::SP_PI;  //rad
constexpr float MAX_J5 = 0.00;       //rad

constexpr float MIN_J0 = -sp::SP_PI;  //rad
constexpr float MIN_J1 = -sp::SP_PI;  //rad
constexpr float MIN_J2 = -sp::SP_PI;  //rad
constexpr float MIN_J3 = -sp::SP_PI;  //rad
constexpr float MIN_J4 = -sp::SP_PI;  //rad
constexpr float MIN_J5 = -0.00;       //rad

constexpr float MID_J0 = -1.0486;           //rad
constexpr float MID_J1 = -2.79673;          //rad
constexpr float MID_J2 = -1.60069;          //rad
constexpr float MID_J3 = -1.0994;           //rad
constexpr float MID_J4 = 0.26476f - 0.69f;  //rad
constexpr float MID_J5 = 0;                 //rad

inline JointMotorController j0_controller(
  MID_J0, MIN_J0, MAX_J0, false, motor_j0, j0_pos_pid, j0_speed_pid);
inline JointMotorController j1_controller(
  MID_J1, MIN_J1, MAX_J1, true, motor_j1, j1_pos_pid, j1_speed_pid);
inline JointMotorController j2_controller(
  MID_J2, MIN_J2, MAX_J2, true, motor_j2, j2_pos_pid, j2_speed_pid);
inline JointMotorController j3_controller(
  MID_J3, MIN_J3, MAX_J3, false, motor_j3, j3_pos_pid, j3_speed_pid);
inline JointMotorController j4_controller(
  MID_J4, MIN_J4, MAX_J4, false, motor_j4, j4_pos_pid, j4_speed_pid);
inline JointMotorController j5_controller(
  MID_J5, MIN_J5, MAX_J5, false, motor_j5, j5_pos_pid, j5_speed_pid);
#endif  // CONTROLLERS_HPP