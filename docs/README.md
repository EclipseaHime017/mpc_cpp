# mpc_cpp — 四足机器人 MPC 控制器 C++ 实现

将 `mpc-control/`（Python + MuJoCo）中的分层控制系统移植到 C++，直接部署到真实四足机器人（Robstride 电机 + WIT-Motion IMU）。

---

## 目录结构

```
mpc_cpp/
├── CMakeLists.txt              # 构建配置（Eigen/OSQP/CasADi/libusb）
├── config/
│   └── robot_params.yaml       # 所有机器人参数（质量、增益、MPC权重等）
├── include/
│   ├── common/
│   │   ├── robot_config.hpp    # 参数结构体，from_yaml() 加载
│   │   ├── robot_state.hpp     # RobotState / MPCOutput / IMUSensorData
│   │   └── math_utils.hpp      # 四元数、旋转矩阵、skew 等（纯头文件）
│   ├── estimator/
│   │   └── state_estimator.hpp # IMU + 运动学互补滤波
│   ├── gait/
│   │   └── gait_generator.hpp  # Trot 步态 + Bezier 摆线轨迹
│   ├── kinematics/
│   │   └── quadruped_kin.hpp   # 三连杆解析式正/逆运动学 + 雅可比
│   ├── mpc/
│   │   ├── mpc_controller.hpp  # Convex MPC（OSQP，10步，12状态）
│   │   └── nmpc_footstep.hpp   # NMPC 落足点优化（CasADi + IPOPT）
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
│   └── test_mpc_solver.cpp     # 离线单元测试（无需硬件）
└── scripts/
    └── install_deps.sh         # 一键安装所有依赖
```

硬件通信层（电机/IMU/手柄）直接复用 `../pure_cpp/Main/`，不重复实现。

---

## 控制架构

```
500 Hz PD 线程（SCHED_FIFO pri=90）        30 Hz MPC 线程
─────────────────────────────────────       ───────────────────────
IMU 读取（WIT serial，已有后台线程）         读最新 RobotState
关节状态读取（CAN，已有后台线程）            NMPC / Convex MPC 求解
状态估计（互补滤波）                         更新 SharedMPCData（mutex）
步态推进（Trot 相位）
WBC（J^T * f_mpc + 摆腿 PD）
发送 MIT 命令（CAN）
```

---

## 依赖

| 库 | 版本 | 用途 |
|----|------|------|
| Eigen3 | ≥ 3.4 | 所有矩阵运算 |
| yaml-cpp | 任意 | 参数加载 |
| OSQP | 0.6.3 | Convex MPC QP 求解 |
| osqp-cpp / OsqpEigen | 任意 | OSQP C++ 封装（二选一）|
| CasADi | ≥ 3.6 | NMPC 非线性优化 |
| IPOPT | ≥ 3.14 | CasADi 内部 NLP 求解器 |
| libusb-1.0 | 任意 | CAN 接口（pure_cpp 依赖）|

---

## 快速开始

### 1. 安装依赖

```bash
bash scripts/install_deps.sh
```

脚本自动安装：Eigen/yaml-cpp/libusb（apt），OSQP/osqp-cpp/CasADi（源码编译）。

### 2. 编译

```bash
cd BFYP/mpc_cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

编译产物：
- `build/mpc_robot_control` — 完整控制器（需要硬件）
- `build/test_mpc_solver`   — 离线单元测试（无需硬件）

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
1. 初始化 4 路 CAN，绑定 12 个电机，进入 MIT 模式
2. 30 步插值到站立姿态（约 3s）
3. IMU 标定（2s 静止，估计零偏）
4. 进入双频率控制循环

手柄控制（`/dev/input/js0`）：
- 轴 1 → 前进速度（±0.5 m/s）
- 轴 0 → 侧移速度（±0.5 m/s）
- 轴 3 → 偏航角速度（±0.5 rad/s）

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
| `mit_kp` / `mit_kd` | 40 / 0.5 | 电机 MIT 位置增益 |

---

## 硬件接线说明

- **CAN**：4 路 `candle0`–`candle3`，各接 3 个电机（HipA/HipF/Knee）
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
