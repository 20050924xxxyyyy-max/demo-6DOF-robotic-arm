#ifndef AUTOMATIONS_HPP
#define AUTOMATIONS_HPP

#include "automations/automation/automation.hpp"
#include "automations/helpers/helpers.hpp"
#include "controllers/controllers.hpp"

constexpr float MIN_F_BLOCK = 5.9;          // N
constexpr float MIN_T_BLOCK = 1;            // N·m
constexpr float MIN_T_GRIPPER_BLOCK = 4.0;  // N·m

constexpr float Z_GOLD_SIDES = -0.562;  // m
constexpr float Z_GOLD_MIDDLE = -0.56;  // m

inline bool arm_z_calibrated = false;
inline bool arm_deployed = false;
inline bool ore_stored = false;
inline bool fpv_calibrated = false;
inline bool gripper_calibrated = false;
// clang-format off

//收臂
inline auto deploy_arm = Sequence(
  move_j2(1.206f),
  // move_j0(-sp::SP_PI),
  move_j1(0.1530f),
  // move_j2(1.5248f),
  // move_j3(0.0f),
  // move_j4(0.0f),
  // move_j5(0.0f),
  // move_j4(-0.4f),
  Call([]() {
    arm_j0.cmd(arm_j0.pos);
    arm_j1.cmd(arm_j1.pos);
    arm_j2.cmd(arm_j2.pos);
    arm_j3.cmd(arm_j3.pos);
    arm_j4.cmd(arm_j4.pos);
    arm_j5.cmd(arm_j5.pos);
    arm_deployed = true;
  })
);

inline auto calibrate_fpv = Sequence(
  // Call([]() {
  //   fpv::cmd_pitch(fpv::MAX_PITCH);
  // }),
  // init_fpv_z(-0.1, MIN_F_BLOCK),
  // move_fpv_z((fpv::MIN_Z + fpv::MAX_Z) / 2),
  // Call([]() {
  //   fpv::cmd_pitch(0);
  //   fpv_calibrated = true;
  // })
);

// clang-format on

#endif  // AUTOMATIONS_HPP