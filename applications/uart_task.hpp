#ifndef UART_TASK_HPP
#define UART_TASK_HPP

#include "referee/vt03/vt03.hpp"

inline sp::VT03 vt03(&huart1, true);

#endif  // UART_TASK_HPP