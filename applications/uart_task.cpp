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

  if (huart == vt03.huart) {
    vt03.update(Size, stamp_ms);
    vt03.request();
  }
}

// 错误
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == vt03.huart) {
    vt03.request();
  }
  if (huart == &huart5) {
    remote.request();
  }
}

// 任务入口
extern "C" void uart_task()
{
  vt03.request();
  remote.request();
  while (true) {
    osDelay(100);
  }
}