#include "control_task.hpp"

#include "automations/automation_manager/automation_manager.hpp"
#include "automations/automations.hpp"
#include "automations/helpers/helpers.hpp"
#include "automations/joint_move/joint_move.hpp"
#include "can.hpp"
#include "cmsis_os.h"
#include "controllers/controllers.hpp"
#include "src/matrix.h"
#include "src/robotics.h"
#include "uart_task.hpp"

extern "C" void control_task()
{
  fdcan1.start();
  fdcan2.start();
  fdcan3.start();

  arm_init_enable();
  while (true) {
    arm_error_detect();
    arm_j0.control();
    arm_j1.control();
    arm_j2.control();
    arm_j3.control();
    arm_j4.control();
    arm_j4.pos = sp::limit_min_max(arm_j4.pos, sp::SP_PI, 2 * sp::SP_PI);
    arm_j5.control();
    send_arm_j0();
    send_arm_j1();
    send_arm_j2();
    send_arm_j3();
    send_arm_j4();
    send_arm_j5();
    osDelay(1);
  }
}