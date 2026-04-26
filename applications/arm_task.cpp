#include "arm_task.hpp"

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

float plot_vel_cmd = 0.0f;

FeedbackMode mode = FeedbackMode::DISABLE;
FeedbackMode last_mode = FeedbackMode::DISABLE;
auto last_remote_sw_l = sp::DBusSwitchMode::DOWN;
extern bool arm_deployed;
AutomationManager automation;
void switch_mode();
float m[6] = {0.40687, 0.4725, 0.43641, 0.42052, 0.37429, 0.0001};  // 质量
// clang-format off
static float rc_data[18] = {
  0.09 * 1e-3, 14.66 * 1e-3,  0.65 * 1e-3,  
  -186.64 * 1e-3, -7.92 * 1e-3,  -0.93 * 1e-3,
  37.35 * 1e-3, 67.25 * 1e-3, -6.61 * 1e-3, 
  0.09 * 1e-3, -5.58 * 1e-3,  215.10 * 1e-3,
  -57.22 * 1e-3, 28.59 * 1e-3, -4.02 * 1e-3,
  -0.00001 * 1e-3,   -0.00001 * 1e-3, -0.00001 * 1e-3};
Matrixf<6, 3> rc_temp(rc_data);
Matrixf<3, 6> rc = rc_temp.trans();

Matrixf<3, 3> I[6]{
  matrixf::diag<3, 3>(
    std::array<float, 3>{353230.51f * 1e-9f, 301232.84f * 1e-9f, 238072.37 * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{195374.20f * 1e-9f, 1701144.97f * 1e-9f, 1662713.69 * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{390577.94f * 1e-9f, 275124.53f * 1e-9f, 372245.81f * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{777989.74f * 1e-9f, 779004.45f * 1e-9f, 150826.81f * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{193744.28f * 1e-9f, 265924.74f * 1e-9f, 242431.65f * 1e-9f}.data()),
  matrixf::eye<3, 3>() * 1e-9f,
};  // 惯性张量

float q[6] = {0};
float qv[6] = {0};
float qa[6] = {0};
float he[6] = {0};
// clang-format on
robotics::Link links[6] = {
  robotics::Link(
    theta[0], d[0], a[0], alpha[0], robotics::R, theta[0], -10000, 10000, m[0], rc.col(0), I[0]),
  robotics::Link(
    theta[1], d[1], a[1], alpha[1], robotics::R, theta[1], -10000, 10000, m[1], rc.col(1), I[1]),
  robotics::Link(
    theta[2], d[2], a[2], alpha[2], robotics::R, theta[2], -10000, 10000, m[2], rc.col(2), I[2]),
  robotics::Link(
    theta[3], d[3], a[3], alpha[3], robotics::R, theta[3], -10000, 10000, m[3], rc.col(3), I[3]),
  robotics::Link(
    theta[4], d[4], a[4], alpha[4], robotics::R, theta[4], -10000, 10000, m[4], rc.col(4), I[4]),
  robotics::Link(
    theta[5], d[5], a[5], alpha[5], robotics::R, theta[5], -10000, 10000, m[5], rc.col(5), I[5]),
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
    // if (!arm_deployed) {
    //   automation.load(&deploy_arm);
    // }
    // arm_j0.cmd(arm_j0.pos);
    // arm_j1.cmd(arm_j1.pos);
    // arm_j2.cmd(arm_j2.pos);
    // arm_j3.cmd(arm_j3.pos);
    // arm_j4.cmd(arm_j4.pos);
    // arm_j5.cmd(arm_j5.pos);
    arm_j0.disable();
    arm_j1.disable();
    arm_j2.disable();
    arm_j3.disable();
    arm_j4.disable();
    arm_j5.disable();
  }
  else if (mode == FeedbackMode::POSITION) {
    if (!arm_deployed) {
      automation.load(&deploy_arm);
    }
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

extern "C" void arm_task()
{
  osDelay(3000);  // 等待全部使能
                  // automation.load(&deploy_arm);（初始化）
  while (true) {
    mode_control();
    if (mode != last_mode) {
      switch_mode();
    }
    calc_grav_t();
    if (mode == FeedbackMode::DISABLE) {
      handle_disable();
    }
    if (mode == FeedbackMode::TORQUE) {
      handle_torque();
    }
    // handle_disable();
    if (mode == FeedbackMode::POSITION) {
      handle_position();
    }
    automation.run();
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

void handle_position()
{
  if (vt03.robot.mode) {
    if (fabs(vt03.robot.j0_pos - arm_j0.pos) > 0.1 && fabs(vt03.robot.j0_vel) < 0.15) {
      arm_j0.cmd(vt03.robot.j0_pos);
    }
    else {
      arm_j0.cmd_t(arm_j0.feedforward_t_);
    }
    if (fabs(vt03.robot.j1_pos - arm_j1.pos) > 0.1 && fabs(vt03.robot.j1_vel) < 0.15) {
      arm_j1.cmd(vt03.robot.j1_pos);
    }
    else {
      arm_j1.cmd_t(arm_j1.feedforward_t_);
    }
    if (fabs(vt03.robot.j2_pos - arm_j2.pos) > 0.1 && fabs(vt03.robot.j2_vel) < 0.15) {
      arm_j2.cmd(vt03.robot.j2_pos);
    }
    else {
      arm_j2.cmd_t(arm_j2.feedforward_t_);
    }
    arm_j3.cmd_t(arm_j3.feedforward_t_);
    arm_j4.cmd_t(arm_j4.feedforward_t_);
    arm_j5.disable();
    // arm_j3.cmd(vt03.robot.j3);
    // arm_j4.cmd(vt03.robot.j4);
  }
  else {
    arm_j0.cmd_t(arm_j0.feedforward_t_);
    arm_j1.cmd_t(arm_j1.feedforward_t_);
    arm_j2.cmd_t(arm_j2.feedforward_t_);
    arm_j3.cmd_t(arm_j3.feedforward_t_);
    arm_j4.cmd_t(arm_j4.feedforward_t_);
    arm_j5.cmd_t(arm_j5.feedforward_t_);
  }
  // if (!automation.idle()) return;

  // arm_j0.add(0.0f);
  // arm_j1.add(0.0f);
  // arm_j2.add(0.0f);
  // arm_j3.add(0.0f);
  // arm_j4.add(0.0f);
  // arm_j5.add(0.0f);
  // arm_j0.cmd_t(arm_j0.feedforward_t_);
  // arm_j1.cmd_t(arm_j1.feedforward_t_);
  // arm_j2.cmd_t(arm_j2.feedforward_t_);
  // arm_j3.cmd_t(arm_j3.feedforward_t_);
  // arm_j4.cmd_t(arm_j4.feedforward_t_);
  // arm_j5.cmd_t(arm_j5.feedforward_t_);

  // //速度环
  // float raw = float(remote.ch_lh);
  // const float scale = 10.0f;  // 调整此值以改变最大速度（示例：660 * 0.005 ≈ 3.3 rad/s）
  // const float max_vel = 3.0f;  // 安全上限（rad/s），根据电机/机械限位调整

  // float vel_cmd = raw * scale;
  // plot_vel_cmd = vel_cmd;
  // vel_cmd = sp::limit_min_max(vel_cmd, -max_vel, max_vel);

  // arm_j3.cmd_v(vel_cmd);

  // //位置环
  // //将遥控通道映射为每个周期的位置增量（rad/tick）
  // float raw_0 = float(remote.ch_rh);  // 例如范围 -660..660
  // const float scale_0 = 0.01f;        // 每个 control loop tick 的增量，按需调小/调大
  // float delta_0 = raw_0 * scale_0;
  // arm_j0.add(delta_0);

  // float raw_1 = float(remote.ch_rv);
  // const float scale_1 = 0.01f;
  // float delta_1 = raw_1 * scale_1;
  // arm_j1.add(delta_1);

  // float raw_2 = float(remote.ch_lv);
  // const float scale_2 = 0.01f;
  // float delta_2 = raw_2 * scale_2;
  // arm_j2.add(delta_2);

  // //arm_j1.cmd_t(arm_j1.feedforward_t_);
  // //arm_j2.cmd_t(arm_j2.feedforward_t_);
  // //arm_j3.cmd_t(arm_j3.feedforward_t_);
  // arm_j4.cmd_t(arm_j4.feedforward_t_);
  // arm_j5.cmd_t(arm_j5.feedforward_t_);

  // arm_j0.disable();
  // arm_j1.disable();
  // arm_j2.disable();
  // arm_j3.disable();
  // arm_j4.disable();
  // arm_j5.disable();
}
//if (!automation.idle()) return; // 若在自动化中则不干扰
//位置环
// 将遥控通道映射为每个周期的位置增量（rad/tick）
// float raw = float(remote.ch_rh);  // 例如范围 -660..660
// const float scale = 0.0005f;      // 每个 control loop tick 的增量，按需调小/调大
// float delta = raw * scale;

// // 增量应用到关节位置设定（会在 JointMotorController 内累加到 set_）
//arm_j0.add(delta);
void handle_torque()
{
  // if (!automation.idle()) return;

  // arm_j0.add(0.0f);
  // arm_j1.add(0.0f);
  // arm_j2.add(0.0f);
  // arm_j3.add(0.0f);
  // arm_j4.add(0.0f);
  // arm_j5.add(0.0f);
  arm_j0.cmd_t(arm_j0.feedforward_t_);
  arm_j1.cmd_t(arm_j1.feedforward_t_);
  arm_j2.cmd_t(arm_j2.feedforward_t_);
  arm_j3.cmd_t(arm_j3.feedforward_t_);
  arm_j4.cmd_t(arm_j4.feedforward_t_);
  arm_j5.cmd_t(arm_j5.feedforward_t_);
  // arm_j0.disable();
  // arm_j1.disable();
  // arm_j2.disable();
  // arm_j3.disable();
  // arm_j4.disable();
  // arm_j5.disable();
}

// void handle_position()
// {
//   // arm_j0.disable();
//   // arm_j1.disable();
//   // arm_j2.disable();
//   // arm_j3.disable();
//   // arm_j4.disable();
//   // arm_j5.disable();
//   if (!automation.idle()) return;
//   arm_j0.add(0.0f);
//   arm_j1.add(0.0f);
//   arm_j2.add(0.0f);
//   arm_j3.add(0.0f);
//   arm_j4.add(0.0f);
//   arm_j5.add(0.0f);
// }