# arm_task.cpp 详细代码分析文档

## 📋 文件概览
**职责**：STM32H7 机械臂的主控制任务，运行在 FreeRTOS 上，1ms 循环。  
**核心功能**：
- 重力补偿计算（RNE 递推牛顿-欧拉法）
- 模式管理（禁用 / 力矩 / 位置）
- 自动化任务执行
- 关节控制

---

## 🔧 全局变量详解

### 控制模式管理
```cpp
FeedbackMode mode = FeedbackMode::DISABLE;           // 当前控制模式
FeedbackMode last_mode = FeedbackMode::DISABLE;      // 上一帧模式
auto last_remote_sw_l = sp::DBusSwitchMode::DOWN;    // 上一帧遥控器开关状态
```
**三种模式**：
- `DISABLE`：禁用所有关节
- `TORQUE`：直接力矩控制（遥控器）
- `POSITION`：位置控制（自动化脚本）

### 外部变量
```cpp
extern bool arm_deployed;                            // 外部导入的臂展开标志
AutomationManager automation;                        // 自动化任务管理器实例
```

---

## 📊 机器人学数据初始化

### 1. 关节质量
```cpp
float m[6] = {0.42072, 0.41367, 0.233403, 0.161971, 0.078939, 0.0001};  // kg
```
- 6 个关节的质量，用于重力补偿计算
- 最后一个关节质量极小（近似 0）

### 2. 质量中心（重心）位置矩阵
```cpp
static float rc_data[18] = {
  0 * 1e-3, -9.281 * 1e-3,  107.324 * 1e-3,     // J0: [x, y, z] (m)
  -19.53 * 1e-3, -0.15 * 1e-3,  107.18 * 1e-3,  // J1
  0 * 1e-3, -42.780 * 1e-3, 72.806 * 1e-3,      // J2
  -1.313 * 1e-3, -8.30 * 1e-3,  -4.23 * 1e-3,   // J3
  0.281 * 1e-3,  -3.864 * 1e-3, 51.008 * 1e-3,  // J4
  -0.00001 * 1e-3, -0.00001 * 1e-3, -0.00001 * 1e-3  // J5 (近似原点)
};
Matrixf<6, 3> rc_temp(rc_data);        // 暂存为 6×3 矩阵
Matrixf<3, 6> rc = rc_temp.trans();    // 转置为 3×6（用于 robotics 库）
```
**说明**：每列代表一个关节的质心位置偏移，单位为米。

### 3. 惯性张量（Inertia Matrix）
```cpp
Matrixf<3, 3> I[6]{
  matrixf::diag<3, 3>({853161.97f * 1e-9f, ...})  // J0 的 3×3 对角惯性张量
  // ...J1~J5 的惯性张量
  matrixf::eye<3, 3>() * 1e-9f                    // J5: 单位矩阵 × 1e-9
};
```
**结构**：`I[i]` 是第 i 个关节的 3×3 对角矩阵，单位为 kg·m²。

---

## 🔗 机器人学建模

### D-H 参数（Denavit-Hartenberg）
```cpp
constexpr float theta[6] = {sp::SP_PI, 0, 0, sp::SP_PI, sp::SP_PI / 2, 0};
constexpr float d[6] = {0, 0, -0.0415, 0.16086, 0, 0};           // 平移 (m)
constexpr float a[6] = {0, 0.116, 0, 0.012, 0, 0};               // 连杆长度 (m)
constexpr float alpha[6] = {sp::SP_PI / 2, 0, -sp::SP_PI / 2, sp::SP_PI / 2, -sp::SP_PI / 2, 0};
```
| 参数 | 含义 | 示例（J0） |
|------|------|----------|
| `theta` | 关节初始角度 | π rad |
| `d` | Z 轴平移 | 0 m |
| `a` | X 轴连杆长度 | 0 m |
| `alpha` | 扭转角 | π/2 rad |

### 关节角度限制
```cpp
constexpr float MAX_J0 = 10000;   // rad（虚拟值）
constexpr float MAX_J1 = 3.2110;  // rad ≈ 184°
constexpr float MAX_J2 = 1.7;     // rad ≈ 97°
// ... 类似的最小值 MIN_J0~J5
```

### 中点位置（回收位置）
```cpp
constexpr float MID_J0 = -0.531;    // rad
constexpr float MID_J1 = -2.79673;  // rad
// ...
```

### 机械臂建模
```cpp
robotics::Link links[6] = {
  robotics::Link(
    theta[0], d[0], a[0], alpha[0],     // D-H 参数
    robotics::R,                         // 回转关节类型
    theta[0], -10000, 10000,             // 初始角，范围
    m[0], rc.col(0), I[0]                // 质量、质心、惯性张量
  ),
  // ...J1~J5
};

robotics::Serial_Link<6> sp_arm(links);  // 创建 6DOF 串联机械臂
```

---

## 📈 状态向量

```cpp
float q[6] = {0};      // 关节位置 (rad)
float qv[6] = {0};     // 关节速度 (rad/s) - 目前为 0
float qa[6] = {0};     // 关节加速度 (rad/s²) - 目前为 0
float he[6] = {0};     // 末端外力 - 目前为 0
```
**注**：`qv`, `qa`, `he` 全为 0，意味着只计算静态重力补偿，不考虑动态和外力。

---

## 🎛️ 主要函数分析

### 1. `mode_control()` - 模式控制
```cpp
void mode_control()
{
  last_mode = mode;
  // 注释掉了遥控器检测逻辑
  // if (remote.sw_r == sp::DBusSwitchMode::DOWN)
  //   mode = FeedbackMode::DISABLE;
  // else if (remote.sw_r == sp::DBusSwitchMode::MID)
  //   mode = FeedbackMode::DISABLE;
  // else if (remote.sw_r == sp::DBusSwitchMode::UP)
  
  mode = FeedbackMode::TORQUE;  // 固定为力矩模式
}
```
**功能**：
- 保存上一帧模式
- 当前代码中固定模式为 `TORQUE`
- 注释的逻辑原本用于检测遥控器开关 `remote.sw_r`

---

### 2. `switch_mode()` - 模式切换处理
```cpp
void switch_mode()
{
  automation.quit();  // 停止任何运行中的自动化任务
  
  if (mode == FeedbackMode::TORQUE) {
    // 力矩模式：禁用所有关节
    arm_j0.disable();
    arm_j1.disable();
    // ...
    arm_j5.disable();
  }
  else if (mode == FeedbackMode::POSITION) {
    // 位置模式
    if (!arm_deployed) {
      automation.load(&deploy_arm);  // 加载展开臂动画
    }
    // 发送当前位置为目标值（保持位置）
    arm_j0.cmd(arm_j0.pos);
    arm_j1.cmd(arm_j1.pos);
    // ...
    arm_j5.cmd(arm_j5.pos);
  }
}
```

**逻辑**：
| 模式 | 行为 | 含义 |
|------|------|------|
| `TORQUE` | 禁用所有关节 | 等待遥控器输入（不会响应，因为禁用了） |
| `POSITION` | 加载自动化任务 + 锁定位置 | 执行脚本或保持当前位置 |

**注**：`DISABLE` 模式在此未处理（应在 `handle_disable()` 中）。

---

### 3. `calc_grav_t()` - 重力补偿计算 ⭐ 核心
```cpp
void calc_grav_t()
{
  // 步骤 1: 从电机反馈读取当前关节角度
  q[0] = arm_j0.pos;
  q[1] = arm_j1.pos;
  // ... q[2]~q[5]

  // 步骤 2: 使用 RNE 算法计算力矩
  Matrixf<6, 1> torq = sp_arm.rne(q, qv, qa, he);  // 参数：位置、速度、加速度、外力

  // 步骤 3: 设置前馈补偿力矩
  arm_j0.set_feedforward(torq[0][0]);  // torq 是 6×1 矩阵
  arm_j1.set_feedforward(torq[1][0]);
  // ...
  arm_j5.set_feedforward(torq[5][0]);
}
```

**RNE 算法说明**：
- **输入**：$q, \dot{q}, \ddot{q}, h_e$ (位置、速度、加速度、末端外力)
- **计算过程**：递推计算每个关节的重力和惯性力矩
- **输出**：$\tau$ (6×1 向量，每个关节所需的力矩)

**公式（静态情况）**：
$$\tau_i = m_i g h_i \cos(q_i + \text{offset})$$

其中 `qv=0, qa=0, he=0`，所以只有 **重力项**。

---

### 4. `arm_task()` - 主控制循环 ⭐ 核心
```cpp
extern "C" void arm_task()
{
  osDelay(3000);  // 等待系统初始化 3 秒
  
  while (true) {
    // 每 1ms 循环一次
    
    // 步骤 1: 检测模式
    mode_control();
    
    // 步骤 2: 模式切换时处理
    if (mode != last_mode) {
      switch_mode();
    }
    
    // 步骤 3: 计算重力补偿
    calc_grav_t();
    
    // 步骤 4: 根据模式执行控制处理
    if (mode == FeedbackMode::DISABLE) {
      handle_disable();
    }
    if (mode == FeedbackMode::TORQUE) {
      handle_remote();
    }
    if (mode == FeedbackMode::POSITION) {
      handle_keyboard();
    }
    
    // 步骤 5: 执行自动化任务
    automation.run();
    
    // 步骤 6: 延迟 1ms（RTOS 时钟节拍）
    osDelay(1);
  }
}
```

**执行时序**：
```
T=0ms:       模式检测 → 重力计算 → 处理函数 → 自动化 → 1ms延迟
T=1ms:       重复...
T=3000ms:    从此开始进入循环
```

---

### 5. `handle_disable()` - 禁用模式处理
```cpp
void handle_disable()
{
  arm_z_calibrated = false;     // 清除 Z 轴校准标志
  arm_deployed = false;         // 臂未展开
  arm_j0.disable();
  arm_j1.disable();
  // ...
  arm_j5.disable();             // 禁用所有电机
}
```
**功能**：断开所有关节，进入自由落体/手动模式。

---

### 6. `handle_remote()` - 遥控器模式处理
```cpp
void handle_remote()
{
  // 注释掉了检查自动化任务的条件
  // if (!automation.idle()) return;

  // 向 6 个关节发送前馈力矩（重力补偿）
  arm_j0.cmd_t(arm_j0.feedforward_t_);  // cmd_t = command torque
  arm_j1.cmd_t(arm_j1.feedforward_t_);
  // ...
  arm_j5.cmd_t(arm_j5.feedforward_t_);
}
```
**功能**：
- 应用前面计算的重力补偿力矩
- 遥控器的直接输入会在电机驱动层叠加

**注**：注释掉的 `automation.idle()` 原本用于检查是否有自动化任务在运行。

---

### 7. `handle_keyboard()` - 键盘/位置模式处理
```cpp
void handle_keyboard()
{
  if (!automation.idle()) return;  // 如果有自动化任务在运行，跳过
  
  arm_j0.add(0.0f);  // 增量输入（当前为 0）
  arm_j1.add(0.0f);
  // ...
  arm_j5.add(0.0f);
}
```
**功能**：
- 检查是否有自动化任务在运行
- 执行微调增量（目前为 0，相当于无操作）

---

## 🔄 完整执行流程图

```
START (arm_task 创建)
  ↓
osDelay(3000ms)  ← 等待系统初始化
  ↓
┌─────────────────────────────────────────────────┐
│ 主循环 (while(true), 1ms 周期)                   │
└─────────────────────────────────────────────────┘
  ↓
1. mode_control()
   └─> 检测模式 (DISABLE/TORQUE/POSITION)
       (当前固定为 TORQUE)
  ↓
2. if (mode != last_mode)?
   ├─ YES → switch_mode()
   │         └─> 清理自动化任务
   │         └─> 根据新模式初始化
   └─ NO → 跳过
  ↓
3. calc_grav_t()  ⭐ 重力补偿计算
   ├─> 读取 q[0...5] (关节位置)
   ├─> 调用 sp_arm.rne(q, qv, qa, he)
   └─> 设置 arm_j0~j5 的前馈力矩
  ↓
4. Mode-specific handlers
   ├─ DISABLE    → handle_disable()    (禁用所有)
   ├─ TORQUE     → handle_remote()     (发送重力补偿)
   └─ POSITION   → handle_keyboard()   (微调)
  ↓
5. automation.run()  (执行自动化任务序列)
  ↓
6. osDelay(1)  (让出 CPU 给其他任务)
  ↓
回到步骤 1 (1ms 后)
```

---

## 💾 关键数据结构关系图

```
arm_task() 主循环
    ↓
当前位置: q[6] = {arm_j0.pos, ..., arm_j5.pos}
    ↓
sp_arm.rne(q, qv=0, qa=0, he=0)  ← robotics 库
    ↓
输出 torq[6] (所需力矩)
    ↓
设置前馈: arm_j0~j5.set_feedforward()
    ↓
JointMotorController 执行
    ├─ 位置 PID 环
    ├─ 速度 PID 环
    ├─ 加上前馈力矩
    └─ 发送 CAN 命令 → 电机
        ↓
    电机反馈 → 下一周期读取
```

---

## 🎯 重要状态标志

```cpp
extern bool arm_deployed;          // 由外部模块管理
inline bool arm_z_calibrated;      // Z 轴是否校准
inline bool arm_deployed;          // 臂是否展开
inline bool ore_stored;            // 矿石是否存储（比赛相关）
inline bool fpv_calibrated;        // FPV 摄像头是否校准
inline bool gripper_calibrated;    // 夹爪是否校准
```

---

## 🚨 当前代码状态

### 被注释的功能
1. **遥控器模式检测**：全部注释，固定使用 `TORQUE` 模式
2. **自动化任务检查**：`handle_remote()` 中的 `if (!automation.idle())` 被注释
3. **禁用模式**：`handle_disable()` 调用被注释

### 建议改进
```cpp
// ✗ 当前：固定 TORQUE 模式
mode = FeedbackMode::TORQUE;

// ✓ 建议：恢复遥控器检测
if (remote.sw_r == sp::DBusSwitchMode::DOWN)
  mode = FeedbackMode::DISABLE;
else if (remote.sw_r == sp::DBusSwitchMode::MID)
  mode = FeedbackMode::DISABLE;  // 可改为另一模式
else if (remote.sw_r == sp::DBusSwitchMode::UP)
  mode = FeedbackMode::POSITION;
```

---

## 📝 参数汇总表

| 参数 | 值 | 单位 | 说明 |
|------|-----|------|------|
| 任务周期 | 1 | ms | osDelay(1) |
| 初始延迟 | 3000 | ms | osDelay(3000) |
| 关节数 | 6 | - | 6DOF 机械臂 |
| 总质量 | ~1.2 | kg | Σ m[i] |
| 最大J1角 | 3.211 | rad | ~184° |
| 最大J2角 | 1.7 | rad | ~97° |
| D-H参数个数 | 4×6 | - | theta/d/a/alpha |
| 状态向量长度 | 4×6 | - | q/qv/qa/he |

---

## 🔗 相关文件/函数调用

```
arm_task.cpp
├── 包含的头文件
│   ├── automations/automation_manager/automation_manager.hpp
│   ├── automations/automations.hpp
│   ├── controllers/controllers.hpp
│   ├── src/robotics.h  ← RNE 算法库
│   ├── src/matrix.h    ← 矩阵库
│   └── uart_task.hpp   ← 遥控器 remote 对象
│
├── 外部调用
│   ├── arm_j0~j5: JointMotorController 对象
│   ├── automation: AutomationManager 对象
│   ├── sp_arm: robotics::Serial_Link<6> 对象
│   └── deploy_arm: 自动化任务序列
│
└── 被调用方式
    └── extern "C" void arm_task()  ← FreeRTOS 任务入口
```

---

## 总结

**arm_task.cpp** 是机械臂控制系统的核心，实现了：
1. ✅ 1ms 高频控制循环
2. ✅ RNE 算法的重力补偿
3. ✅ 三模式控制（禁用/力矩/位置）
4. ✅ 自动化任务集成
5. ✅ 关节 PID 控制前馈补偿

**当前状态**：模式切换逻辑被禁用，固定为 TORQUE 模式，可能处于调试或开发阶段。
