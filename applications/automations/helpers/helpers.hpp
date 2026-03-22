#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <functional>

#include "automations/automation/automation.hpp"
#include "automations/joint_move/joint_move.hpp"

JointMove move_z(float z);
JointMove move_fpv_z(float z);

JointMove move_j0(float j0_set);
JointMove move_j1(float j1_set);
JointMove move_j2(float j2_set);
JointMove move_j3(float j3_set);
JointMove move_j4(float j4_set);
JointMove move_j5(float j5_set);

// Parallel move_arm(float x, float y, float z, float yaw, float roll, float pitch);

Sequence init_joint(
  float v, float effort_thresh, const float * vel, const float * effort,
  std::function<void(float)> cmd_v, std::function<void(void)> init);

Sequence init_z(float v, float effort_thresh);
Sequence init_fpv_z(float v, float effort_thresh);
Sequence init_gripper(float v, float effort_thresh);

#endif  // HELPERS_HPP