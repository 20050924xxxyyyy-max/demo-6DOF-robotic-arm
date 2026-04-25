#ifndef SEND_TASK_HPP
#define SEND_TASK_HPP

#include <cstddef>
#include <cstdint>

#include "can.hpp"
#include "referee/referee_protocol/referee_protocol.hpp"

constexpr size_t CUSTOM_DATA_LEN =
  30;  // 数据段长度固定为30, ref: RoboMaster 裁判系统串口协议附录 V1.7.0（20241225）
constexpr uint32_t TRANSMIT_TIMEOUT = 50;  // ms

// Controller class
#pragma pack(1)
class Controller
{
private:
  struct __attribute__((packed))
  {
    sp::referee::FrameHeader head;
    uint16_t cmd_id;
    sp::referee::CustomRobotData data;
    uint16_t tail;  //for crc16
  } frame_;

public:
  float j0 = 0;       // rad
  float j1 = 0;       // rad
  float j2 = 0;       // rad
  float j3 = 0;       // rad
  float j4 = 0;       // rad
  float j5 = 0;       // rad

  void update();
  void head_set();
  void pack_data();
  void apply_crc();
  void send_data();
};
#pragma pack()

#endif  // SEND_TASK_HPP
