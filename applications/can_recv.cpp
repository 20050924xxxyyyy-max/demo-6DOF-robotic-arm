#include "can.hpp"
#include "controllers/controllers.hpp"
#include "cmsis_os.h"

extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef * hfdcan, uint32_t RxFifo0ITs)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
    if (hfdcan == &hfdcan1) {
      fdcan1.recv();

      if (fdcan1.rx_id == motor_j0.rx_id)
        motor_j0.read(fdcan1.rx_data, stamp_ms);
      else if (fdcan1.rx_id == motor_j1.rx_id)
        motor_j1.read(fdcan1.rx_data, stamp_ms);
      else if (fdcan1.rx_id == motor_j2.rx_id)
        motor_j2.read(fdcan1.rx_data, stamp_ms);
      else if (fdcan1.rx_id == motor_j3.rx_id)
        motor_j3.read(fdcan1.rx_data, stamp_ms);
      else if (fdcan1.rx_id == motor_j4.rx_id)
        motor_j4.read(fdcan1.rx_data, stamp_ms);
      else if (fdcan1.rx_id == motor_j5.rx_id)
        motor_j5.read(fdcan1.rx_data, stamp_ms);
    }
  }
}

void CAN_bus_off_check_reset(FDCAN_HandleTypeDef * hfdcan)
{
  FDCAN_ProtocolStatusTypeDef protocolStatus = {};
  HAL_FDCAN_GetProtocolStatus(hfdcan, &protocolStatus);
  if (protocolStatus.BusOff) {
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
  }
}

extern "C" void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef * hfdcan, uint32_t ErrorStatusITs)
{
  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != RESET) {
    CAN_bus_off_check_reset(hfdcan);
  }
}