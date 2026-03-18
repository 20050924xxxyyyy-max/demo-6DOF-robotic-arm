#include "can_task.hpp"

extern "C" void can_task()
{
  can1.config();
  can1.start();

  while (true) {
    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor_roll2.rx_id)
        motor_roll2.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_pitch.rx_id)
        motor_pitch.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_roll1.rx_id)
        motor_roll1.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_yaw.rx_id)
        motor_yaw.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_x.rx_id)
        motor_x.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_y.rx_id)
        motor_y.read(can1.rx_data, stamp_ms);
      else if (can1.rx_id == motor_z.rx_id)
        motor_z.read(can1.rx_data, stamp_ms);
    }
  }
}