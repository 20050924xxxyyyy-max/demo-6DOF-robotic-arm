#include "cmsis_os.h"
#include "controllers/controllers.hpp"
#include "io/plotter/plotter.hpp"

sp::Plotter plotter(&huart7, false);
extern float roll2;

extern "C" void plotter_task()
{
  while (true) {
    plotter.plot(
      motor_j0.angle, motor_j1.angle, motor_j2.angle, motor_j3.angle, motor_j4.angle,
      motor_j5.angle, arm_j0.pos, arm_j1.pos, arm_j2.pos, arm_j3.pos, arm_j4.pos, arm_j5.pos);
    osDelay(10);  // 100Hz
  }
}