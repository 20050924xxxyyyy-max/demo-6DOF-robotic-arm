#include "pump_led_task.hpp"

#include "send_task/send_task.hpp"

bool pump = false;

PUMP_LED pump_led(&htim1);

PUMP_LED::PUMP_LED(TIM_HandleTypeDef * htim) : htim_(htim) {}

void PUMP_LED::start()
{
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_2);  // dismiss return
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_3);  // dismiss return
  HAL_TIM_PWM_Start(htim_, TIM_CHANNEL_4);  // dismiss return
}

// 设置LED颜色, r/g/b取值范围为[0, 1]
void PUMP_LED::set(float r, float g, float b)
{
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_2, (htim_->Instance->ARR + 1) * r);
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_3, (htim_->Instance->ARR + 1) * g);
  __HAL_TIM_SET_COMPARE(htim_, TIM_CHANNEL_4, (htim_->Instance->ARR + 1) * b);
}

extern "C" void pump_led_task()
{
  pump_led.start();

  while (true) {
    pump = HAL_GPIO_ReadPin(PUMP_GPIO_Port, PUMP_Pin) ? false : true;

    if (pump) {
      for (int i = 0; i < 3; i++) {
        pump_led.set(0.8, 0.8, 1);
        osDelay(50);
        pump_led.set(0.16, 0.16, 0.2);
        osDelay(50);
        pump_led.set(0.8, 0.8, 1);
        osDelay(50);
      }
    }
    else {
      pump_led.set(1, 0, 1);
      osDelay(10);
    }
  }
}