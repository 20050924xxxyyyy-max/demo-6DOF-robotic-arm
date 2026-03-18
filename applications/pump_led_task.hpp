#ifndef PUMP_LED_TASK_HPP
#define PUMP_LED_TASK_HPP

#include "cmsis_os.h"
#include "tim.h"

class PUMP_LED
{
public:
  PUMP_LED(TIM_HandleTypeDef * htim);

  void start();

  // 取值范围: [0, 1]
  void set(float r, float g, float b);  // 从C2开始, PWM接口的顺序

private:
  TIM_HandleTypeDef * htim_;
};

#endif  // PUMP_LED_TASK_HPP
