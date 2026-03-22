#ifndef UART_TASK_HPP
#define UART_TASK_HPP

#include "io/dbus/dbus.hpp"
#include "referee/vt03/vt03.hpp"

inline sp::VT03 vt03(&huart1, true);
inline sp::DBus remote(&huart5, false);
#endif  // UART_TASK_HPP