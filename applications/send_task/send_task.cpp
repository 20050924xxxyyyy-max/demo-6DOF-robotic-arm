#include "send_task.hpp"

#include "can.hpp"
#include "cmsis_os.h"
#include "controllers/controllers.hpp"
#include "main.h"
#include "motor/rm_motor/rm_motor.hpp"
#include "string.h"
#include "tools/crc/crc.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "usart.h"

extern bool arm_deployed;
Controller tx_data;

// 校准零点偏移
void Controller::offset_init()
{
  if (this->has_initiated) return;
  this->j0_offset = motor_j0.angle;
  this->j1_offset = motor_j1.angle;
  this->j2_offset = motor_j2.angle;
  this->j3_offset = motor_j3.angle;
  this->j4_offset = motor_j4.angle;
  this->j5_offset = motor_j5.angle;
  if (
    this->j0_offset && this->j1_offset && this->j2_offset && this->j3_offset && this->j4_offset &&
    this->j5_offset)
    has_initiated = true;  // 确定已经校准
}

float j4_limited = 0;
// 从电机读取当前姿态位置
void Controller::update()
{
  this->control = true;
  this->j0 = sp::limit_min_max(sp::limit_angle(arm_j0.pos), -2.8f, 2.8f);
  this->j1 = arm_j1.pos;
  this->j2 = arm_j2.pos;
  this->j3 = arm_j3.pos;
  this->j4 = arm_j4.pos;
  this->j5 = arm_j5.vel;
  this->gripper = 0;  // TODO gripper
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
  float buff_[8] = {this->control, this->j0, this->j1, this->j2,
                    this->j3,      this->j4, this->j5, this->gripper};
  // float buff_[7] = {0, 1, 2, 0, 0, 0, 0};
  memcpy(this->frame_.data.data, buff_, 8 * sizeof(float));
  // memcpy(this->frame_.data.data + 8 * sizeof(float), &this->pump, sizeof(bool));
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