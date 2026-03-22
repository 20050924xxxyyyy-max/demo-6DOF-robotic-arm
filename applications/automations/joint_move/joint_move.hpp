#ifndef JOINT_MOVE_HPP
#define JOINT_MOVE_HPP

#include <functional>

#include "automations/automation/automation.hpp"

constexpr float MAX_P_ERROR = 1e-2;      // m
constexpr float MAX_ANGLE_ERROR = 6e-2;  // rad
constexpr float MAX_V_STILL = 1e-2;      // rad/s
constexpr float MAX_W_STILL = 1e-2;      // rad/s

class JointMove : public Automation
{
public:
  JointMove(
    float value, const float * pos, const float * vel, std::function<void(float)> cmd,
    bool relative = false, bool prismatic = false);
  bool run() override;
  void quit() override;

private:
  const float value_;
  const float * pos_;
  const float * vel_;
  std::function<void(float)> cmd_;
  const bool relative_;
  const bool prismatic_;
  bool has_cmd_ = false;
  float set_ = 0;
};

class JointMoveUntil : public Automation
{
public:
  JointMoveUntil(
    float vel_set, float effort_thresh, const float * vel, const float * force,
    std::function<void(float)> cmd_v);
  bool run() override;
  void quit() override;

private:
  const float vel_set_;
  const float effort_thresh_;
  const float * vel_;
  const float * force_;
  bool has_cmd_ = false;
  std::function<void(float)> cmd_v_;
};

class JointMoveUntil2 : public Automation
{
public:
  JointMoveUntil2(
    float vel_set, float effort_thresh1, float effort_thresh2, const float * vel,
    const float * force1, const float * force2, std::function<void(float)> cmd_v);
  bool run() override;
  void quit() override;

private:
  const float vel_set_;
  const float effort_thresh1_;
  const float effort_thresh2_;
  const float * vel_;
  const float * force1_;
  const float * force2_;
  bool has_cmd_ = false;
  std::function<void(float)> cmd_v_;
};

#endif  //JOINT_MOVE_HPP