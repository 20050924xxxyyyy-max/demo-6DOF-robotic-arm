#include "control_task.hpp"

#include "automations/automation_manager/automation_manager.hpp"
#include "automations/automations.hpp"
#include "automations/helpers/helpers.hpp"
#include "automations/joint_move/joint_move.hpp"
#include "can.hpp"
#include "cmsis_os.h"
#include "controllers/controllers.hpp"
#include "src/matrix.h"
#include "src/robotics.h"
#include "uart_task.hpp"

FeedbackMode mode;
FeedbackMode last_mode;
auto last_remote_sw_l = sp::DBusSwitchMode::DOWN;
bool deployed = false;
AutomationManager automation;
void switch_mode();
float m[6] = {0.42072, 0.41367, 0.233403, 0.161971, 0.078939, 0.0001};  // 质量
// clang-format off
static float rc_data[18] = {
  0 * 1e-3, -9.281 * 1e-3,  107.324 * 1e-3,  
  -19.53 * 1e-3, -0.15 * 1e-3,  107.18 * 1e-3,
  0 * 1e-3, -42.780 * 1e-3, 72.806 * 1e-3, 
  -1.313 * 1e-3, -8.30 * 1e-3,  -4.23 * 1e-3,
  0.281 * 1e-3,  -3.864 * 1e-3, 51.008 * 1e-3,
  -0.00001 * 1e-3,   -0.00001 * 1e-3, -0.00001 * 1e-3};
Matrixf<6, 3> rc_temp(rc_data);
Matrixf<3, 6> rc = rc_temp.trans();

Matrixf<3, 3> I[6]{
  matrixf::diag<3, 3>(
    std::array<float, 3>{853161.97f * 1e-9f, 767319.00f * 1e-9f, 283012.83 * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{200577.99f * 1e-9f, 813684.68f * 1e-9f, 793261.99 * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{895449.75f * 1e-9f, 272040.73f * 1e-9f, 678932.37f * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{83776.83f * 1e-9f, 82648.23f * 1e-9f, 16432.15f * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{216462.11f * 1e-9f, 128603.9f * 1e-9f, 229525.98f * 1e-9f}.data()),
  matrixf::eye<3, 3>() * 1e-9f,
};  // 惯性张量

float q[6] = {0};
float qv[6] = {0};
float qa[6] = {0};
float he[6] = {0};
// clang-format on
robotics::Link links[6] = {
  robotics::Link(
    theta[0], d[0], a[0], alpha[0], robotics::R, theta[0], MIN_J0, MAX_J0, m[0], rc.col(0), I[0]),
  robotics::Link(
    theta[1], d[1], a[1], alpha[1], robotics::R, theta[1], MIN_J1, MAX_J1, m[1], rc.col(1), I[1]),
  robotics::Link(
    theta[2], d[2], a[2], alpha[2], robotics::R, theta[2], MIN_J2, MAX_J2, m[2], rc.col(2), I[2]),
  robotics::Link(
    theta[3], d[3], a[3], alpha[3], robotics::R, theta[3], MIN_J3, MAX_J3, m[3], rc.col(3), I[3]),
  robotics::Link(
    theta[4], d[4], a[4], alpha[4], robotics::R, theta[4], MIN_J4, MAX_J4, m[4], rc.col(4), I[4]),
  robotics::Link(
    theta[5], d[5], a[5], alpha[5], robotics::R, theta[5], MIN_J5, MAX_J5, m[5], rc.col(5), I[5]),
};

robotics::Serial_Link<6> sp_arm(links);

void mode_control()
{
  last_mode = mode;
  if (remote.sw_r == sp::DBusSwitchMode::DOWN)
    mode = FeedbackMode::DISABLE;
  else if (remote.sw_r == sp::DBusSwitchMode::MID)
    mode = FeedbackMode::TORQUE;
  else if (remote.sw_r == sp::DBusSwitchMode::UP)
    mode = FeedbackMode::POSITION;
}

void switch_mode()
{
  automation.quit();
  if (mode == FeedbackMode::TORQUE) {
    arm_j0.cmd(arm_j0.pos);
    arm_j1.cmd(arm_j1.pos);
    arm_j2.cmd(arm_j2.pos);
    arm_j3.cmd(arm_j3.pos);
    arm_j4.cmd(arm_j4.pos);
    arm_j5.cmd(arm_j5.pos);
  }
  else if (mode == FeedbackMode::POSITION) {
    automation.load(&deploy_arm);
    arm_j0.cmd(arm_j0.pos);
    arm_j1.cmd(arm_j1.pos);
    arm_j2.cmd(arm_j2.pos);
    arm_j3.cmd(arm_j3.pos);
    arm_j4.cmd(arm_j4.pos);
    arm_j5.cmd(arm_j5.pos);
  }
}

void calc_grav_t()
{
  q[0] = arm_j0.pos;
  q[1] = arm_j1.pos;
  q[2] = arm_j2.pos;
  q[3] = arm_j3.pos;
  q[4] = arm_j4.pos;
  q[5] = arm_j5.pos;

  Matrixf<6, 1> torq = sp_arm.rne(q, qv, qa, he);  // 解算

  arm_j0.set_feedforward(torq[0][0]);
  arm_j1.set_feedforward(torq[1][0]);
  arm_j2.set_feedforward(torq[2][0]);
  arm_j3.set_feedforward(torq[3][0]);
  arm_j4.set_feedforward(torq[4][0]);
  arm_j5.set_feedforward(torq[5][0]);
}

extern "C" void control_task()
{
  fdcan1.start();
  fdcan2.start();
  fdcan3.start();

  arm_init_enable();
  remote.request();
  while (true) {
    arm_error_detect();
    mode_control();
    if (mode != last_mode) {
      switch_mode();
    }
    calc_grav_t();
    if (mode == FeedbackMode::DISABLE) {
      handle_disable();
    }
    if (mode == FeedbackMode::TORQUE) {
      handle_remote();
    }
    // handle_disable();
    if (mode == FeedbackMode::POSITION) {
      handle_keyboard();
    }
    automation.run();
    arm_j0.control();
    arm_j1.control();
    arm_j2.control();
    arm_j3.control();
    arm_j4.control();
    arm_j5.control();
    send_arm_j0();
    send_arm_j1();
    send_arm_j2();
    send_arm_j3();
    send_arm_j4();
    send_arm_j5();
    osDelay(1);
  }
}

void handle_disable()
{
  arm_z_calibrated = false;
  arm_deployed = false;
  arm_j0.disable();
  arm_j1.disable();
  arm_j2.disable();
  arm_j3.disable();
  arm_j4.disable();
  arm_j5.disable();
}

void handle_remote()
{
  if (!automation.idle()) return;

  if (remote.sw_l == sp::DBusSwitchMode::DOWN) {
    arm_j0.cmd_t(arm_j0.feedforward_t_);
    arm_j1.cmd_t(arm_j1.feedforward_t_);
    arm_j2.cmd_t(arm_j2.feedforward_t_);
    arm_j3.cmd_t(arm_j3.feedforward_t_);
    arm_j4.cmd_t(arm_j4.feedforward_t_);
    arm_j5.disable();
  }
  else if (remote.sw_l == sp::DBusSwitchMode::MID) {
    // arm_j0.add(-remote.ch_lh * 0.01);
    arm_j0.add(0.0f);
    arm_j1.add(remote.ch_lv * 0.01);
    // arm_j2.add(remote.ch_rv * 0.03);
    // arm_j3.add(remote.ch_rh * 0.03);
    // arm_j1.cmd_t(arm_j1.feedforward_t_);
    // arm_j2.cmd_t(arm_j2.feedforward_t_);
    arm_j2.add(remote.ch_rv * 0.03);
    // arm_j1.cmd(arm_j1.pos);
    // arm_j2.cmd(arm_j2.pos);
    // arm_j3.cmd(arm_j3.pos);
    arm_j3.disable();
    // arm_j4.cmd(arm_j4.pos);
    // arm_j4.add(0.0f);
    arm_j4.disable();
    arm_j5.disable();
  }
  else if (remote.sw_l == sp::DBusSwitchMode::UP) {
    // arm_j0.cmd(arm_j0.pos);
    // arm_j1.cmd(arm_j1.pos);
    // arm_j2.cmd(arm_j2.pos);
    // arm_j3.cmd(arm_j3.pos);
    arm_j0.add(0.0f);
    arm_j1.add(0.0f);
    arm_j2.add(0.0f);
    arm_j3.add(0.0f);
    arm_j4.add(remote.ch_lv * 0.03);
    arm_j5.add(remote.ch_lh * 0.03);
  }
  last_remote_sw_l = remote.sw_l;
}

void handle_keyboard()
{
  // arm_j0.disable();
  // arm_j1.disable();
  // arm_j2.disable();
  // arm_j3.disable();
  // arm_j4.disable();
  // arm_j5.disable();
  if (!automation.idle()) return;
  arm_j0.add(0.0f);
  arm_j1.add(0.0f);
  arm_j2.add(0.0f);
  arm_j3.add(0.0f);
  arm_j4.add(0.0f);
  arm_j5.add(0.0f);
}