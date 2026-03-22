#include "helpers.hpp"

#include "controllers/controllers.hpp"

JointMove move_j0(float j0_set)
{
  return {j0_set, &arm_j0.pos, &arm_j0.vel, [](auto value) { arm_j0.cmd(value); }, false, false};
}

JointMove move_j1(float j1_set)
{
  return {j1_set, &arm_j1.pos, &arm_j1.vel, [](auto value) { arm_j1.cmd(value); }, false, false};
}

JointMove move_j2(float j2_set)
{
  return {j2_set, &arm_j2.pos, &arm_j2.vel, [](auto value) { arm_j2.cmd(value); }, false, false};
}

JointMove move_j3(float j3_set)
{
  return {j3_set, &arm_j3.pos, &arm_j3.vel, [](auto value) { arm_j3.cmd(value); }, false, false};
}

JointMove move_j4(float j4_set)
{
  return {j4_set, &arm_j4.pos, &arm_j4.vel, [](auto value) { arm_j4.cmd(value); }, false, false};
}

JointMove move_j5(float j5_set)
{
  return {j5_set, &arm_j5.pos, &arm_j5.vel, [](auto value) { arm_j5.cmd(value); }, false, false};
}

Sequence init_joint(
  float v, float effort_thresh, const float * vel, const float * effort,
  std::function<void(float)> cmd_v, std::function<void(void)> init)
{
  auto call = Call([v, cmd_v] { cmd_v(v); });
  auto wait = Wait([effort_thresh, vel, effort] {
    return std::abs(*vel) < MAX_V_STILL && std::abs(*effort) > effort_thresh;
  });
  auto call2 = Call([cmd_v] { cmd_v(0); });
  auto call3 = Call([init] { init(); });
  return {call, wait, call2, call3};
}
