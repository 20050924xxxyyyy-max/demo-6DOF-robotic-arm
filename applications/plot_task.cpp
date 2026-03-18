#include "cmsis_os.h"
#include "io/plotter/plotter.hpp"

sp::Plotter plotter(&huart6, false);
extern float roll2;

extern "C" void plot_task()
{
  while (true) {
    // plotter.plot(roll2, 0);
    osDelay(10);  // 100Hz
  }
}