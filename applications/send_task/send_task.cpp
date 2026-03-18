#include "send_task.hpp"

#include "can_task.hpp"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "main.h"
#include "motor/rm_motor/rm_motor.hpp"
#include "string.h"
#include "tools/crc/crc.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "usart.h"

Controller tx_data;

// 校准零点偏移
void Controller::offset_init()
{
  if (this->has_initiated) return;
  this->yaw_offset = motor_yaw.angle;
  this->roll1_offset = motor_roll1.angle;
  this->pitch_offset = motor_pitch.angle;
  if (this->yaw_offset && this->roll1_offset && this->pitch_offset)
    has_initiated = true;  // 确定已经校准
}

// 从电机读取当前姿态位置
void Controller::update()
{
  this
  this->roll1 = sp::limit_angle(motor_roll1.angle - this->roll1_offset);
  this->yaw = -sp::limit_angle(motor_yaw.angle - this->yaw_offset);
  this->pitch = sp::limit_angle(motor_pitch.angle - this->pitch_offset);

  this->x = AMP_X * this->x + X_OFFSET;
  this->y = AMP_Y * this->y;
  this->z = AMP_Z * this->z;
  this->roll2 = AMP_ROLL2 * this->roll2;

  this->roll1 = sp::limit_min_max(AMP_ROLL1 * this->roll1, -sp::PI, sp::PI);
  this->yaw = sp::limit_min_max(AMP_YAW * this->yaw, -sp::PI, sp::PI);
  this->pitch = sp::limit_min_max(AMP_PITCH * this->pitch, -sp::PI, sp::PI);

  this->pump = HAL_GPIO_ReadPin(PUMP_GPIO_Port, PUMP_Pin) ? false : true;
}

// 设置裁判系统协议帧头
void Controller::head_set()
{
  static uint8_t seq = 0;
  // 帧头信息设置
  this->frame_.head.sof = sp::referee::SOF;                      // 设置帧头起始符
  this->frame_.head.data_len = CUSTOM_DATA_LEN;                  // 设置数据长度
  this->frame_.head.seq = seq++;                                 // 设置序列号
  this->frame_.cmd_id = sp::referee::cmd_id::CUSTOM_ROBOT_DATA;  // 设置命令ID
}

// 将数据打包到协议帧中
void Controller::pack_data()
{
  float buff_[7] = {this->yaw, this->roll1, this->pitch, this->roll2, this->x, this->y, this->z};
  memcpy(this->frame_.data.data, buff_, 7 * sizeof(float));
  memcpy(this->frame_.data.data + 7 * sizeof(float), &this->pump, sizeof(bool));
}

// 计算并设置协议帧的CRC校验码
void Controller::apply_crc()
{
  // 先计算crc8
  auto head_len = sizeof(this->frame_.head);
  auto crc8_len = sizeof(this->frame_.head.crc8);
  frame_.head.crc8 =
    sp::get_crc8(reinterpret_cast<uint8_t *>(&this->frame_.head), head_len - crc8_len);

  // 再计算crc16
  auto len = sizeof(this->frame_);
  auto tail_len = sizeof(this->frame_.tail);
  this->frame_.tail = sp::get_crc16(reinterpret_cast<uint8_t *>(&this->frame_), len - tail_len);
}

// 通过UART发送协议帧
void Controller::send_data()
{
  HAL_UART_Transmit(&huart1, (uint8_t *)(&this->frame_), sizeof(this->frame_), TRANSMIT_TIMEOUT);
}

extern "C" void send_task()
{
  while (true) {
    tx_data.offset_init();
    tx_data.update();
    tx_data.head_set();
    tx_data.pack_data();
    tx_data.apply_crc();
    tx_data.send_data();

    osDelay(33);  // 30Hz
  }
}