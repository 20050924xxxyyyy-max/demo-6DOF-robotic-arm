#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

sp::Buzzer buzzer(&htim12, TIM_CHANNEL_2, 240e6);

constexpr int P0 = 0;  // 休止符频率

constexpr int L1 = 262;  // 低音频率
constexpr int L2 = 294;
constexpr int L3 = 330;
constexpr int L4 = 349;
constexpr int L5 = 392;
constexpr int L6 = 440;
constexpr int L7 = 494;

constexpr int M1 = 523;  // 中音频率
constexpr int M2 = 587;
constexpr int M3 = 659;
constexpr int M4 = 698;
constexpr int M5 = 784;
constexpr int M6 = 880;
constexpr int M7 = 988;

constexpr int H1 = 1047;  // 高音频率
constexpr int H2 = 1175;
constexpr int H3 = 1319;
constexpr int H4 = 1397;
constexpr int H5 = 1568;
constexpr int H6 = 1760;
constexpr int H7 = 1976;

extern "C" void buzzer_task()
{
  for (int i = 0; i < 2; i++) {
    buzzer.start();
    buzzer.set(H1, 0.7);
    osDelay(100);
    buzzer.set(H3, 0.7);
    osDelay(100);
    buzzer.set(H5, 0.7);
    osDelay(100);
    buzzer.stop();
    osDelay(100);
  }

  while (true) {
    osDelay(1000);
  }

  return;
}