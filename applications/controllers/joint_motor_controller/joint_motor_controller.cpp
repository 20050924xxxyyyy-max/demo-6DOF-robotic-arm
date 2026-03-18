#include "joint_motor_controller.hpp"

#include "tools/math_tools/math_tools.hpp"

JointMotorController::JointMotorController(
  float mid, float min, float max, bool reverse, sp::DM_Motor & motor, sp::PID & pid,
  sp::PID & motor_speed_pid, bool feedforward)
: mid_(mid),
  min_(min),
  max_(max),
  sign_(reverse ? -1 : 1),
  feedforward_(feedforward),
  motor_(motor),
  pid_(pid),
  motor_speed_pid_(motor_speed_pid)
{
}

void JointMotorController::disable() { mode_ = ControlMode::DISABLE; }

void JointMotorController::add(float value) { cmd(sp::limit_angle(set_ + value)); }

void JointMotorController::cmd(float value)
{
  mode_ = ControlMode::POSITION;
  set_ = sp::limit_min_max(value, min_, max_);
}

void JointMotorController::cmd_v(float value)
{
  mode_ = ControlMode::VELOCITY;
  v_set_ = value;
}

void JointMotorController::cmd_t(float value)
{
  mode_ = ControlMode::TORQUE;
  t_set_ = sign_ * value;
}

void JointMotorController::set_feedforward(float value)
{
  this->feedforward_t_ = feedforward_ ? value : 0;
}

void JointMotorController::control()
{
  this->pos = sign_ * sp::limit_angle(sp::limit_angle(motor_.angle) - mid_);
  this->vel = sign_ * motor_.speed;
  this->torque_fdb = sign_ * motor_.torque;

  if (mode_ == ControlMode::DISABLE) {
    motor_.cmd(0);
    return;
  }
  if (mode_ == ControlMode::TORQUE) {
    motor_.cmd(t_set_);
  }
  else {
    if (mode_ == ControlMode::POSITION) {
      pid_.calc(set_, pos);
      motor_speed_pid_.calc(pid_.out, this->vel);
    }
    else if (mode_ == ControlMode::VELOCITY) {
      motor_speed_pid_.calc(v_set_, this->vel);
    }
    torque_cmd =
      sign_ * (feedforward_ ? motor_speed_pid_.out + feedforward_t_ : motor_speed_pid_.out);
    motor_.cmd(torque_cmd);
  }
}