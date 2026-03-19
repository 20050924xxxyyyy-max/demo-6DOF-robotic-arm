#include "control_task.hpp"

#include "can.hpp"
#include "cmsis_os.h"
#include "controllers/controllers.hpp"
#include "src/matrix.h"
#include "src/robotics.h"
#include "uart_task.hpp"

FeedbackMode mode;

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

  arm_init_enable();
  while (true) {
    mode_control();
    if (mode == FeedbackMode::DISABLE) handle_disable();
    if (mode == FeedbackMode::TORQUE) {
      // TODO feedback 1    strategy
      j0_controller.cmd_t(j0_controller.feedforward_t_);
      j1_controller.cmd_t(j1_controller.feedforward_t_);
      j2_controller.cmd_t(j2_controller.feedforward_t_);
      j3_controller.cmd_t(j3_controller.feedforward_t_);
      j4_controller.cmd_t(j4_controller.feedforward_t_);
      j5_controller.cmd_t(j5_controller.feedforward_t_);
    }
    j0_controller.control();
    j1_controller.control();
    j2_controller.control();
    j3_controller.control();
    j4_controller.control();
    j5_controller.control();
    send_arm_j0();
    send_arm_j1();
    send_arm_j2();
    send_arm_j3();
    send_arm_j4();
    send_arm_j5();
    osDelay(1);
  }
}
