#ifndef ARM_TASK_HPP
#define ARM_TASK_HPP

// 两种工作模式
enum class FeedbackMode
{
  DISABLE,
  TORQUE,
  POSITION
};
extern FeedbackMode mode;
extern FeedbackMode last_mode;
void handle_disable();
void handle_remote();
void handle_keyboard();
#endif  // ARM_TASK_HPP