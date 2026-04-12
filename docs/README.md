# mpc_cpp — 四足机器人 MPC 控制器 C++ 实现

将 `mpc-control/`（Python + MuJoCo）中的分层控制系统移植到 C++，直接部署到真实四足机器人（Robstride 电机 + WIT-Motion IMU）。

---

## 目录结构

```
mpc_cpp/
├── CMakeLists.txt              # 构建配置（Eigen/OSQP/CasADi/libusb）
├── config/
│   └── robot_params.yaml       # 所有机器人参数（质量、增益、MPC权重等）
├── driver/                     # 硬件通信层（解耦自 pure_cpp/Main/）
│   ├── include/
│   │   ├── robstride.hpp       # Robstride 电机控制（MIT 模式）
│   │   ├── can_interface.hpp   # CAN 总线通信
│   │   ├── observations.hpp    # IMUComponent / Gamepad / JointComponent
│   │   ├── wit_c_sdk.h         # WIT-Motion IMU SDK
│   │   ├── REG.h               # WIT 寄存器定义
│   │   └── serial.h            # 串口通信
│   └── src/
│       ├── robstride.cpp       # 电机 MIT 命令：SendMITCommand(pos, vel, kp, kd, tau_ff)
│       ├── can_interface.cpp   # CAN 帧收发
│       ├── observations.cpp    # IMU/Gamepad 后台线程
│       ├── serial.c            # 串口底层
│       └── wit_c_sdk.c         # WIT SDK 解析
├── include/
│   ├── common/
│   │   ├── robot_config.hpp    # 参数结构体，from_yaml() 加载
│   │   ├── robot_state.hpp     # RobotState / MPCOutput / IMUSensorData
│   │   └── math_utils.hpp      # 四元数、旋转矩阵、skew 等（纯头文件）
│   ├── estimator/
│   │   └── state_estimator.hpp # IMU + 运动学互补滤波
│   ├── gait/
│   │   └── gait_generator.hpp  # Trot 步态 + Bezier 摆线 + Raibert 启发
│   ├── kinematics/
│   │   └── quadruped_kin.hpp   # 三连杆解析式正/逆运动学 + 雅可比
│   ├── mpc/
│   │   ├── mpc_controller.hpp  # Convex MPC（OSQP，10步，12状态）
│   │   └── nmpc_footstep.hpp   # NMPC 落足点优化（CasADi + IPOPT，暂停使用）
│   └── wbc/
│       └── wbc_controller.hpp  # 全身控制器（雅可比转置法）
├── src/
│   ├── common/robot_config.cpp
│   ├── estimator/state_estimator.cpp
│   ├── gait/gait_generator.cpp
│   ├── kinematics/quadruped_kin.cpp
│   ├── mpc/mpc_controller.cpp
│   ├── mpc/nmpc_footstep.cpp
│   ├── wbc/wbc_controller.cpp
│   └── main.cpp                # 主控制循环（500Hz PD + 30Hz MPC 双线程）
├── tests/
│   ├── test_mpc_solver.cpp     # 离线单元测试（无需硬件）
│   ├── test_single_joint.cpp   # 单关节硬件测试
│   └── test_imu.cpp            # IMU 验证工具
└── docs/
    └── deployment_guide.md     # 部署指南
```

---

## 程序状态机

程序从启动到关机经历以下状态转换：

```
                         ┌─────────────────┐
                         │   INIT (初始化)   │
                         │  加载 YAML 配置   │
                         │  初始化运动学模块  │
                         └────────┬─────────┘
                                  │
                                  v
                     ┌────────────────────────┐
                     │    PHASE A: 电机启动     │
                     │  初始化 4×CAN 总线       │
                     │  绑定 12 个电机 (Enable)  │
                     │  SetMITParams(kp,kd)     │
                     │  30 步插值到站立姿态      │
                     └────────────┬─────────────┘
                                  │
                        Ctrl+C ───┤──> SHUTDOWN
                                  │
                                  v
                     ┌────────────────────────┐
                     │  等待用户确认 (ENTER)    │
                     │  select() 100ms 轮询    │
                     └────────────┬─────────────┘
                                  │
                        Ctrl+C ───┤──> SHUTDOWN
                                  │ ENTER
                                  v
                     ┌────────────────────────┐
                     │  启动 IMU + Gamepad      │
                     │  创建控制模块             │
                     │  启动 MPC 线程 (30Hz)    │
                     │  设置 SCHED_FIFO pri=90  │
                     └────────────┬─────────────┘
                                  │
                                  v
┌─────────────────────────────────────────────────────────┐
│                  500 Hz 主控制循环                        │
│                                                          │
│  ┌───────────────────────────────────────────────┐       │
│  │ 每帧 (2ms):                                    │       │
│  │  1. 读 IMU + 关节状态                          │       │
│  │  2. 关节限位安全检查 ──超限──> EMERGENCY STOP   │       │
│  │  3. 状态估计 (互补滤波)                        │       │
│  │  4. 读 Gamepad → v_cmd                        │       │
│  │  5. 同步 v_cmd 到 MPC 线程                     │       │
│  └──────────────┬────────────────────────────────┘       │
│                 │                                         │
│     ┌───────────┴───────────┐                            │
│     │                       │                            │
│     v                       v                            │
│  t < calib_duration      t >= calib_duration             │
│  ┌─────────────────┐    ┌────────────────────────────┐   │
│  │  PHASE B: 标定    │    │  PHASE C: MPC 控制          │   │
│  │  PD 站立保持      │    │                              │   │
│  │  q_des = 0       │    │  6. 读 MPC 缓存 (mutex)     │   │
│  │  kp = kp_stand   │    │  7. 步态更新 (Trot 相位)    │   │
│  │  kd = kd_stand   │    │  8. WBC: tau_ff = J^T*f_mpc │   │
│  │  tau_ff = 0      │    │  9. 计算电机目标:            │   │
│  │                  │    │     Stance: q=站立角,        │   │
│  │  标定足端偏移     │    │       kp=30, kd=2, tau_ff   │   │
│  │  (t ≈ calib_dur)│    │     Swing: q=摆动轨迹,      │   │
│  └────────┬────────┘    │       kp=150, kd=8, tau_ff=0│   │
│           │              │  10. SendMITCommand(全参数)  │   │
│           │              └────────────┬───────────────┘   │
│           │                           │                   │
│           └───────────────────────────┘                   │
│                         │                                 │
│                  发送 MIT 命令到电机                        │
│    电机内部执行: τ = kp*(q_des-q) + kd*(0-dq) + tau_ff    │
│                                                          │
└──────────────────────────┬───────────────────────────────┘
                           │
                  Ctrl+C ──┤
                  限位超限 ─┤
                           v
                  ┌─────────────────┐
                  │   SHUTDOWN       │
                  │  停止 MPC 线程    │
                  │  发送零位命令     │
                  │  DisableMotor    │
                  └─────────────────┘


  并行线程:
  ┌────────────────────────────────────────────────┐
  │  MPC 线程 (30 Hz, 普通优先级)                    │
  │                                                 │
  │  1. 读 SharedMPCData (v_cmd, gait_phase)       │
  │  2. 读 shared_state (mutex)                     │
  │  3. 同步步态相位                                 │
  │  4. Raibert 启发式落足点规划:                    │
  │     p_foot = p_hip + v * T_stance/2             │
  │  5. Convex MPC (OSQP) → GRF                    │
  │  6. 写回 SharedMPCData (f_stance, foot_target) │
  └────────────────────────────────────────────────┘

  后台硬件线程 (由 driver/ 管理):
  ┌────────────────────────────────────────────────┐
  │  CAN 接收线程 × 4  — 解析电机反馈 (pos/vel/τ)   │
  │  电机在线监测       — 500ms 超时检测/重连        │
  │  IMU 串口线程       — 100μs 周期读取四元数/陀螺  │
  │  Gamepad 线程       — 读取手柄轴/按钮           │
  └────────────────────────────────────────────────┘
```

### MIT 命令帧结构

电机通过 CAN 总线接收 MIT 模式命令，在电机端 (~10 kHz) 执行力矩混合：

```
τ_motor = kp * (pos_des - pos) + kd * (vel_des - vel) + tau_ff

CAN 扩展帧 (29-bit ID + 8 bytes data):
┌──────────────────────────────────────────────────────┐
│ CAN ID (29 bit):                                      │
│   Bits 24-28: msg_type (0x01 = MIT)                   │
│   Bits  8-23: tau_ff   (16-bit, [-T_MAX, T_MAX])     │
│   Bits  0-7:  motor_id                                │
├──────────────────────────────────────────────────────┤
│ Data (8 bytes):                                       │
│   [0-1] pos_des  (16-bit, [-12.57, 12.57] rad)       │
│   [2-3] vel_des  (16-bit, [-44.0, 44.0] rad/s)       │
│   [4-5] kp       (16-bit, [0, 500])                  │
│   [6-7] kd       (16-bit, [0, 5])                    │
└──────────────────────────────────────────────────────┘

Knee 齿轮比转换 (G = 1.667):
  pos_motor = pos_joint * G + offset
  kp_motor  = kp_joint  / G^2
  kd_motor  = kd_joint  / G^2
  tau_motor = tau_joint  / G
```

---

## 控制架构

```
500 Hz 主线程 (SCHED_FIFO pri=90)           30 Hz MPC 线程
──────────────────────────────────          ─────────────────────────
IMU 读取 (WIT serial 后台线程)               读最新 RobotState (mutex)
关节状态读取 (CAN 后台线程)                   同步步态相位
状态估计 (互补滤波)                           Raibert 启发式落足点
步态推进 (Trot 相位)                          Convex MPC (OSQP) → GRF
WBC (tau_ff = J^T * f_mpc)                    更新 SharedMPCData (mutex)
计算 q_des + kp/kd + tau_ff
SendMITCommand (全参数 CAN 帧)

力矩控制链路:
  MPC → GRF (世界系 12 维)
    → WBC: tau_ff = J^T * f_mpc (关节空间 12 维)
      → MIT 帧: (q_des, vel=0, kp, kd, tau_ff)
        → 电机内部: τ = kp*(q_des-q) + kd*(0-dq) + tau_ff
```

---

## 依赖

| 库 | 版本 | 用途 |
|----|------|------|
| Eigen3 | >= 3.4 | 所有矩阵运算 |
| yaml-cpp | 任意 | 参数加载 |
| OSQP | 0.6.3 | Convex MPC QP 求解 |
| osqp-cpp / OsqpEigen | 任意 | OSQP C++ 封装（二选一）|
| CasADi | >= 3.6 | NMPC 非线性优化（预留） |
| IPOPT | >= 3.14 | CasADi 内部 NLP 求解器 |
| libusb-1.0 | 任意 | CAN 接口 |

---

## 快速开始

### 1. 安装依赖

```bash
bash scripts/install_deps.sh
```

脚本自动安装：Eigen/yaml-cpp/libusb（apt），OSQP/OsqpEigen/CasADi（源码编译）。

### 2. 编译

```bash
cd mpc_cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

编译产物：
- `build/mpc_robot_control` — 完整控制器（需要硬件）
- `build/test_mpc_solver`   — 离线单元测试（无需硬件）
- `build/test_single_joint` — 单关节硬件测试
- `build/test_imu`          — IMU 验证工具

### 3. 离线测试（无硬件）

```bash
./build/test_mpc_solver config/robot_params.yaml
```

测试内容：
1. **Kinematics** — 正运动学 / 雅可比合理性
2. **Gait Generator** — 步态相位 + MPC 参考轨迹维度
3. **Convex MPC** — OSQP 收敛 + 法向力符号 + 总力平衡
4. **NMPC Footstep** — IPOPT 收敛 + 落足点在工作空间内

### 4. 上机部署

```bash
# 需要 root（实时线程 + CAN 接口）
sudo ./build/mpc_robot_control config/robot_params.yaml
```

启动流程：
1. **Phase A** — 初始化 4 路 CAN，绑定 12 个电机，30 步插值到站立姿态
2. **等待 ENTER** — 确认机器人已安全站立（Ctrl+C 可中断）
3. **Phase B** — IMU 标定（2s 静止），PD 高刚度站立，标定足端偏移
4. **Phase C** — 进入 500Hz+30Hz 双频率 MPC 控制循环

安全机制：
- 每帧检查 12 个关节限位，超限 0.1 rad 立即紧急停机
- tau_ff 硬限到 `mit_torque_limit` (17 Nm)
- kp/kd 硬限到 Robstride 寄存器范围 (kp: 0~500, kd: 0~5)
- Ctrl+C 在任意阶段均可中断

手柄控制（`/dev/input/js0`）：
- 轴 1 → 前进速度（+/-0.5 m/s）
- 轴 0 → 侧移速度（+/-0.5 m/s）
- 轴 3 → 偏航角速度（+/-0.5 rad/s）

---

## 关键参数（`config/robot_params.yaml`）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `mass` | 12.0 kg | 机器人总质量 |
| `target_z` | 0.20 m | 目标站立高度 |
| `gait_period` | 0.4 s | Trot 步态周期 |
| `step_height` | 0.06 m | 摆腿高度 |
| `mpc_horizon` | 10 | MPC 预测步数 |
| `mu` | 0.5 | 摩擦系数 |
| `f_max` | 150 N | 单腿最大法向力 |
| `kp_stance` / `kd_stance` | 30 / 2 | 支撑腿 PD 增益（柔顺跟踪） |
| `kp_swing` / `kd_swing` | 150 / 8 | 摆动腿 PD 增益（精确跟踪） |
| `mit_torque_limit` | 17 Nm | 电机硬件力矩上限 |

---

## 硬件接线说明

- **CAN**：4 路 `can0`--`can3`，各接 3 个电机（HipA/HipF/Knee）
- **电机 ID**：HipA=[1,5,9,13]，HipF=[2,6,10,14]，Knee=[3,7,11,15]
- **IMU**：`/dev/ttyCH341USB0`，波特率自动扫描
- **手柄**：`/dev/input/js0`（可选）

---

## 与 Python 原型的对应关系

| Python 文件 | C++ 对应 |
|------------|---------|
| `mpc_controller.py` | `src/mpc/mpc_controller.cpp` |
| `nmpc_planner.py` | `src/mpc/nmpc_footstep.cpp` |
| `state_estimator.py` | `src/estimator/state_estimator.cpp` |
| `gait_generator.py` | `src/gait/gait_generator.cpp` |
| `wbc_controller.py` | `src/wbc/wbc_controller.cpp` |
| `mpc_sim.py`（主循环） | `src/main.cpp` |
| 硬件驱动（pure_cpp） | `driver/` |
