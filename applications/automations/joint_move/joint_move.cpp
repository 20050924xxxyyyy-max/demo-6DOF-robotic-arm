#include "automations/joint_move/joint_move.hpp"
#include "tools/math_tools/math_tools.hpp"
JointMove::JointMove(
  float value, const float * pos, const float * vel, std::function<void(float)> cmd, bool relative,
  bool prismatic)
: value_(value), pos_(pos), vel_(vel), cmd_(cmd), relative_(relative), prismatic_(prismatic)
{
}

bool JointMove::run()
{
  if (!has_cmd_) {
    set_ = relative_ ? value_ + *pos_ : value_;
    cmd_(set_);
    has_cmd_ = true;
  }
  return std::abs(sp::limit_angle(set_ - *pos_)) < (prismatic_ ? MAX_P_ERROR : MAX_ANGLE_ERROR) &&
         std::abs(*vel_) < (prismatic_ ? MAX_V_STILL : MAX_W_STILL);
}

void JointMove::quit() { has_cmd_ = false; }

JointMoveUntil::JointMoveUntil(
  float vel_set, float effort_thresh, const float * vel, const float * force,
  std::function<void(float)> cmd_v)
: vel_set_(vel_set), effort_thresh_(effort_thresh), vel_(vel), force_(force), cmd_v_(cmd_v)
{
}

bool JointMoveUntil::run()
{
  cmd_v_(vel_set_);
  if (std::abs(*force_) > effort_thresh_) {
    cmd_v_(0);
    return true;
  }
}

void JointMoveUntil::quit() { cmd_v_(0); }

JointMoveUntil2::JointMoveUntil2(
  float vel_set, float effort_thresh1, float effort_thresh2, const float * vel,
  const float * force1, const float * force2, std::function<void(float)> cmd_v)
: vel_set_(vel_set),
  effort_thresh1_(effort_thresh1),
  effort_thresh2_(effort_thresh2),
  vel_(vel),
  force1_(force1),
  force2_(force2),
  cmd_v_(cmd_v)
{
}

bool JointMoveUntil2::run()
{
  cmd_v_(vel_set_);
  if ((std::abs(*force1_) > effort_thresh1_) || (std::abs(*force2_) > effort_thresh2_)) {
    cmd_v_(0);
    return true;
  }
}

void JointMoveUntil2::quit() { cmd_v_(0); }