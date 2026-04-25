#include "cmsis_os.h"
#include "control_task.hpp"
#include "controllers/controllers.hpp"
#include "io/plotter/plotter.hpp"
#include "send_task/send_task.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "uart_task.hpp"

sp::Plotter plotter(&huart7, false);
extern float roll2;
extern Controller tx_data;
extern float j4_limited;
extern float plot_vel_cmd;
extern "C" void plotter_task()
{
  while (true) {
    //plotter.plot(
      // motor_j0.angle, motor_j1.angle, motor_j2.angle, motor_j3.angle, motor_j4.angle,
      // motor_j5.angle, arm_j0.pos, arm_j1.pos, arm_j2.pos, arm_j3.pos, arm_j4.pos, arm_j5.pos
      // arm_j5.vel, tx_data.j5, arm_j5.pos, arm_j5.torque_fdb, 1.0
      // arm_j5.pos, motor_j5.angle
     // tx_data.j0, tx_data.j1, tx_data.j2, tx_data.j3, tx_data.j4, tx_data.j5
    //);
    // float(remote.sw_r),float(mode),float(last_mode));
    // plotter.plot(
    //   arm_j0.set_, arm_j0.pos, sp::limit_angle(arm_j0.set_ - arm_j0.pos), arm_j0.pid_.out,
    //   arm_j0.motor_speed_pid_.out);
    // plotter.plot(
    //   arm_j1.set_, arm_j1.pos, sp::limit_angle(arm_j1.set_ - arm_j1.pos), arm_j1.pid_.out,
    //   arm_j1.vel, arm_j1.motor_speed_pid_.out);
    // plotter.plot(
    //   arm_j2.set_, arm_j2.pos, sp::limit_angle(arm_j2.set_ - arm_j2.pos), arm_j2.pid_.out,
    //   arm_j2.vel, arm_j2.motor_speed_pid_.out);
    //plotter.plot(arm_j0.pos, arm_j1.pos, arm_j2.pos, arm_j3.pos, arm_j4.pos, arm_j5.pos);


    // plotter.plot(
    //   arm_j2.set_, arm_j2.pos, sp::limit_angle(arm_j2.set_ - arm_j2.pos)
    // );

    plotter.plot(
      plot_vel_cmd,
      arm_j3.vel,
      //arm_j0.motor_speed_pid_.data.pout,
      //arm_j0.motor_speed_pid_.data.dout,
      arm_j0.motor_speed_pid_.out
    );

    osDelay(10);  // 100Hz
  }
}