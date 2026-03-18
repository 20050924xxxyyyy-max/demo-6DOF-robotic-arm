#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

sp::Plotter plotter(&huart7, false);
extern float roll2;

extern "C" void plotter_task()
{
  while (true) {
    // plotter.plot(roll2, 0);
    osDelay(10);  // 100Hz
  }
}