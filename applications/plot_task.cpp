#include "cmsis_os.h"
#include "controllers/controllers.hpp"
#include "io/plotter/plotter.hpp"
#include "tools/math_tools/math_tools.hpp"
sp::Plotter plotter(&huart7, false);
extern float roll2;

extern "C" void plotter_task()
{
  while (true) {
    // plotter.plot(
    //   motor_j0.angle, motor_j1.angle, motor_j2.angle, motor_j3.angle, motor_j4.angle,
    //   motor_j5.angle, arm_j0.pos, arm_j1.pos, arm_j2.pos, arm_j3.pos, arm_j4.pos, arm_j5.pos);
    // plotter.plot(
    //   arm_j0.set_, arm_j0.pos, sp::limit_angle(arm_j0.set_ - arm_j0.pos), arm_j0.pid_.out,
    //   arm_j0.motor_speed_pid_.out);
    // plotter.plot(
    //   arm_j1.set_, arm_j1.pos, sp::limit_angle(arm_j1.set_ - arm_j1.pos), arm_j1.pid_.out,
    //   arm_j1.vel, arm_j1.motor_speed_pid_.out);
    plotter.plot(
      arm_j2.set_, arm_j2.pos, sp::limit_angle(arm_j2.set_ - arm_j2.pos), arm_j2.pid_.out,
      arm_j2.vel, arm_j2.motor_speed_pid_.out);
    osDelay(10);  // 100Hz
  }
}