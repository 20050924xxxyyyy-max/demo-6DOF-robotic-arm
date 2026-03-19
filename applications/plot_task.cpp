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
      motor_j5.angle, j0_controller.pos, j1_controller.pos, j2_controller.pos, j3_controller.pos,
      j4_controller.pos, j5_controller.pos);
    osDelay(10);  // 100Hz
  }
}