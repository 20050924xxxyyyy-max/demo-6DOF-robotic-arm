#ifndef PIDS_HPP
#define PIDS_HPP
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

constexpr float MAX_J0_V = 20;                        // rad/s
constexpr float MAX_J1_V = 2.5;                       // rad/s
constexpr float MAX_J2_V = 2.5;                       // rad/s
constexpr float MAX_J3_V = 3;                         // rad/s
constexpr float MAX_J4_V = 3;                         // rad/s
constexpr float MAX_J5_V = sp::M2006_P36_MAX_TORQUE;  // rad/s

constexpr float MAX_J0_T = 10;                        // N·m
constexpr float MAX_J1_T = 10;                        // N·m
constexpr float MAX_J2_T = 10;                        // N·m
constexpr float MAX_J3_T = 10;                        // N·m
constexpr float MAX_J4_T = 10.0f;                     // N·m
constexpr float MAX_J5_T = sp::M2006_P36_MAX_TORQUE;  // N·m

constexpr float T = 0.001;  // PID控制周期, 单位: s

inline sp::PID j0_speed_pid(T, 0.8, 20, 0.001, MAX_J0_T, 0.5);
inline sp::PID j1_speed_pid(T, 0.8, 20, 0.001, MAX_J1_T, 0.5);
inline sp::PID j2_speed_pid(T, 0.8, 20, 0.001, MAX_J2_T, 0.5);
inline sp::PID j3_speed_pid(T, 0.8, 20, 0.001, MAX_J3_T, 0.5);
inline sp::PID j4_speed_pid(T, 0.8, 20, 0.001, MAX_J4_T, 0.5);
inline sp::PID j5_speed_pid(T, 0.8, 0, 0, MAX_J5_T, MAX_J5_T);
inline sp::PID gripper_speed_pid(T, 0.005, 0, 0, 0.25, 0.01);

//可以整周转动的转劣弧
inline sp::PID j0_pos_pid(T, 20, 0, 0, MAX_J0_V, MAX_J0_V, 1.0f, true);
inline sp::PID j1_pos_pid(T, 10, 0, 0, MAX_J1_V, MAX_J1_V, 1.0f, true);
inline sp::PID j2_pos_pid(T, 10, 0, 0, MAX_J2_V, MAX_J2_V, 1.0f, true);
inline sp::PID j3_pos_pid(T, 20, 0, 0, MAX_J3_V, MAX_J3_V, 1.0f, true);
inline sp::PID j4_pos_pid(T, 20, 0, 0, MAX_J4_V, MAX_J4_V, 1.0f, true);
inline sp::PID j5_pos_pid(T, 0, 0, 0, MAX_J5_V, MAX_J5_V);

#endif  // PIDS_HPP