#include "control_task.hpp"

#include "can.hpp"
#include "cmsis_os.h"
#include "controllers/controllers.hpp"
#include "uart_task.hpp"

FeedbackMode mode;

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
    }
    osDelay(100);
  }
}
