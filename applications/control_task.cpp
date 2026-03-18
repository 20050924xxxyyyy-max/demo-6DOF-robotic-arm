#include "control_task.hpp"

#include "can.hpp"
#include "cmsis_os.h"
#include "controllers/controllers.hpp"
#include "src/matrix.h"
#include "src/robotics.h"
#include "uart_task.hpp"

FeedbackMode mode;

float m[6] = {1.90645, 1.74289, 1.71288, 0.84843, 0.60935, 0.696};  // 质量

static float rc_data[18] = {
  -0.04 * 1e-3, -0.56 * 1e-3,  3.34 * 1e-3,  -142.65 * 1e-3, 0.02 * 1e-3,   -23.17 * 1e-3,
  -2.05 * 1e-3, -28.74 * 1e-3, 82.85 * 1e-3, -25.43 * 1e-3,  -95.41 * 1e-3, 8.35 * 1e-3,
  -0.03 * 1e-3, -3.45 * 1e-3,  -6.37 * 1e-3, 0.042 * 1e-3,   -0.156 * 1e-3, -116.561 * 1e-3};
Matrixf<6, 3> rc_temp(rc_data);
Matrixf<3, 6> rc = rc_temp.trans();

Matrixf<3, 3> I[6]{
  matrixf::diag<3, 3>(
    std::array<float, 3>{2938662.32f * 1e-9f, 2646275.06f * 1e-9f, 3741970.63 * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{2745013.36f * 1e-9f, 18666652.63f * 1e-9f, 17899511.89 * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{15673091.09f * 1e-9f, 13376862.94f * 1e-9f, 4044424.22f * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{3263263.13f * 1e-9f, 1232294.41f * 1e-9f, 2811101.16f * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{216462.11f * 1e-9f, 128603.9f * 1e-9f, 229525.98f * 1e-9f}.data()),
  matrixf::diag<3, 3>(
    std::array<float, 3>{2364.322f * 1e-9f, 2133.539f * 1e-9f, 451.009f * 1e-9f}.data()),
};  // 惯性张量

float q[6] = {0};
float qv[6] = {0};
float qa[6] = {0};
float he[6] = {0};

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
  if (vt03.mode == sp::VT03Mode::C)
    mode = FeedbackMode::DISABLE;
  else if (vt03.mode == sp::VT03Mode::N)
    mode = FeedbackMode::TORQUE;
}

void handle_disable()
{
  motor_j0.cmd(0);
  motor_j1.cmd(0);
  motor_j2.cmd(0);
  motor_j3.cmd(0);
  motor_j4.cmd(0);
  motor_j5.cmd(0);
}

void calc_grav_t()
{
  q[0] = j0_controller.pos;
  q[1] = j1_controller.pos;
  q[2] = j2_controller.pos;
  q[3] = j3_controller.pos;
  q[4] = j4_controller.pos;
  q[5] = j5_controller.pos;

  Matrixf<6, 1> torq = sp_arm.rne(q, qv, qa, he);  // 解算

  j0_controller.set_feedforward(torq[0][0]);
  j1_controller.set_feedforward(torq[1][0]);
  j2_controller.set_feedforward(torq[2][0]);
  j3_controller.set_feedforward(torq[3][0]);
  j4_controller.set_feedforward(torq[4][0]);
  j5_controller.set_feedforward(torq[5][0]);
}

extern "C" void control_task()
{
  fdcan1.start();
  fdcan2.start();
  fdcan3.start();

  while (true) {
    mode_control();
    if (mode == FeedbackMode::DISABLE) handle_disable();
    if (mode == FeedbackMode::TORQUE) {
      // TODO feedback strategy
      j0_controller.cmd_t(j0_controller.feedforward_t_);
      j1_controller.cmd_t(j1_controller.feedforward_t_);
      j2_controller.cmd_t(j2_controller.feedforward_t_);
      j3_controller.cmd_t(j3_controller.feedforward_t_);
      j4_controller.cmd_t(j4_controller.feedforward_t_);
      j5_controller.cmd_t(j5_controller.feedforward_t_);
    }
    osDelay(100);
  }
}
