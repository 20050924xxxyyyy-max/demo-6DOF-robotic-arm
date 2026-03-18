#ifndef CAN_TASK_HPP
#define CAN_TASK_HPP

#include "cmsis_os.h"
#include "io/fdcan/fdcan.hpp"

inline sp::FDCAN fdcan1(&hfdcan1);
inline sp::FDCAN fdcan2(&hfdcan2);
inline sp::FDCAN fdcan3(&hfdcan3);

#endif  // CAN_TASK_HPP