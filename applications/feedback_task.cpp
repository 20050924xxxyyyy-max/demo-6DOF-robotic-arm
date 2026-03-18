#include "feedback_task.hpp"

#include "can_task.hpp"
#include "cmsis_os.h"
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
  motor_x.cmd(0);
  motor_y.cmd(0);
  motor_z.cmd(0);
  motor_yaw.cmd(0);
  motor_roll1.cmd(0);
  motor_pitch.cmd(0);
  motor_roll2.cmd(0);
}

extern "C" void feedback_task()
{
  while (true) {
    mode_control();
    if (mode == FeedbackMode::DISABLE) handle_disable();
    if (mode == FeedbackMode::TORQUE) {
      // TODO feedback strategy
    }
    osDelay(100);
  }
}
