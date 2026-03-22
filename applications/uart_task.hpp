#ifndef UART_TASK_HPP
#define UART_TASK_HPP

#include "io/dbus/dbus.hpp"
// #include "referee/vt03/vt03.hpp"
#include "referee/vt02/vt02.hpp"

// inline sp::VT03 vt03(&huart1, true);
inline sp::VT02 vt02(&huart1);
#endif  // UART_TASK_HPP