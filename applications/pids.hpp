#ifndef PIDS_HPP
#define PIDS_HPP
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

constexpr float MAX_J0_V = 20;   // rad/s
constexpr float MAX_J1_V = 2.5;  // rad/s
constexpr float MAX_J2_V = 5.0;  // rad/s
constexpr float MAX_J3_V = 3;    // rad/s
constexpr float MAX_J4_V = 3;    // rad/s
constexpr float MAX_J5_V = 1;    // rad/s

constexpr float MAX_J0_T = 5;     // N·m
constexpr float MAX_J1_T = 5;     // N·m
constexpr float MAX_J2_T = 5;     // N·m
constexpr float MAX_J3_T = 0.3;   // N·m
constexpr float MAX_J4_T = 0.0f;  // N·m
constexpr float MAX_J5_T = 0.0f;  // N·m

constexpr float T = 0.001;  // PID控制周期, 单位: s

inline sp::PID j0_speed_pid(T, 3.5, 0, 0.0005, MAX_J0_T, 0.5);
inline sp::PID j1_speed_pid(T, 0.8, 0, 0.001, MAX_J1_T, 0.5);
inline sp::PID j2_speed_pid(T, 0.8, 0, 0.001, MAX_J2_T, 0.5);
inline sp::PID j3_speed_pid(T, 0.8, 20, 0.001, MAX_J3_T, 0.5);
inline sp::PID j4_speed_pid(T, 0.8, 20, 0.001, MAX_J4_T, 0.5);
inline sp::PID j5_speed_pid(T, 0.8, 0, 0, MAX_J5_T, MAX_J5_T);
inline sp::PID gripper_speed_pid(T, 0.005, 0, 0, 0.25, 0.01);

//可以整周转动的转劣弧
inline sp::PID j0_pos_pid(T, 30, 0, 0.005, MAX_J0_V, 1, 1.0f, true);
inline sp::PID j1_pos_pid(T, 20, 3, 0.005, MAX_J1_V, 1, 1.0f, true);
inline sp::PID j2_pos_pid(T, 20, 3, 0.005, MAX_J2_V, 1, 1.0f);
inline sp::PID j3_pos_pid(T, 20, 0, 0, MAX_J3_V, MAX_J3_V, 1.0f, true);
inline sp::PID j4_pos_pid(T, 20, 0, 0, MAX_J4_V, MAX_J4_V, 1.0f, true);
inline sp::PID j5_pos_pid(T, 0, 0, 0, MAX_J5_V, MAX_J5_V);

#endif  // PIDS_HPP