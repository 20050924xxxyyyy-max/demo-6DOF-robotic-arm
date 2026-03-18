#ifndef SEND_TASK_HPP
#define SEND_TASK_HPP

#include <cstddef>
#include <cstdint>

#include "referee/referee_protocol/referee_protocol.hpp"

constexpr size_t CUSTOM_DATA_LEN =
  30;  // 数据段长度固定为30, ref: RoboMaster 裁判系统串口协议附录 V1.7.0（20241225）
constexpr uint32_t TRANSMIT_TIMEOUT = 50;  // ms

// constexpr float AMP_ = 3.0f;      // x轴映射放大系数
// constexpr float AMP_Y = 3.0f;      // y轴映射放大系数
// constexpr float AMP_Z = 2.0f;      // z轴映射放大系数
// constexpr float AMP_YAW = 2.5f;    // yaw映射放大系数
// constexpr float AMP_ROLL1 = 2.5f;  // roll1映射放大系数
// constexpr float AMP_PITCH = 2.5f;  // pitch映射放大系数
// constexpr float AMP_ROLL2 = 1.0f;  // roll2映射放大系数

// constexpr float X_OFFSET = 0.05f;  // x偏移量 m

// constexpr float RATIO_XYZ = 0.00665f;  // rad to m

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

  float j0 = 0;       // rad
  float j1 = 0;       // rad
  float j2 = 0;       // rad
  float j3 = 0;       // rad
  float j4 = 0;       // rad
  float j5 = 0;       // rad
  float gripper = 0;  // rad

  bool has_initiated = false;
  float j0_offset = 0;
  float j1_offset = 0;
  float j2_offset = 0;
  float j3_offset = 0;
  float j4_offset = 0;
  float j5_offset = 0;
  float gripper_offset = 0;

public:
  void offset_init();
  void update();
  void head_set();
  void pack_data();
  void apply_crc();
  void send_data();
};
#pragma pack()

#endif  // SEND_TASK_HPP
