#include "uart_task.hpp"

#include "cmsis_os.h"

// 接收中断回调
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();
  if (huart == &huart5) {
    remote.update(Size, stamp_ms);
    remote.request();
  }

  if (huart == &huart1) {
    vt02.update();
    vt02.request();
  }
}

// 错误
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart1) {
    vt02.request();
  }
  if (huart == &huart5) {
    remote.request();
  }
}

// 任务入口
extern "C" void uart_task()
{
  vt02.request();
  remote.request();
  while (true) {
    osDelay(100);
  }
}