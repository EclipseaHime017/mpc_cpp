# 部署与测试优化指南

> 适用平台：搭载 Robstride 电机（CAN 总线）+ WIT-Motion IMU 的四足机器人
> 嵌入式平台：**Jetson Orin NX**（aarch64，Ubuntu 20.04）
> 对应代码：`BFYP/mpc_cpp/`

**图例**
- `[开发机]` — 在开发/调试用的 x86 机器上执行
- `[Jetson]` — 在 Jetson Orin NX 上执行
- ⚠️ — 容易出错的注意事项

---

## 目录

1. [目标平台环境准备](#一目标平台环境准备)
2. [获取源码](#二获取源码)
3. [硬件逐项验证](#三硬件逐项验证)
4. [分阶段功能测试](#四分阶段功能测试)
5. [参数调优流程](#五参数调优流程)
6. [上机检查清单](#六上机检查清单)
7. [常见问题速查](#七常见问题速查)

---

## 一、目标平台环境准备

> `[Jetson]` 所有步骤均在 Jetson 上执行。
> ⚠️ **不能直接复制开发机上的可执行文件**——Jetson 是 aarch64，开发机通常是 x86_64，二进制不兼容，必须在 Jetson 上重新编译。

### 1.1 确认平台架构

```bash
# [Jetson]
uname -m          # 应输出 aarch64
cat /etc/os-release
gcc --version     # 需要 GCC 9 以上
```

### 1.2 安装系统依赖

```bash
# [Jetson]
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake git \
    libeigen3-dev libyaml-cpp-dev \
    libusb-1.0-0-dev coinor-libipopt-dev \
    can-utils iproute2
```

> `coinor-libipopt-dev` 是 CasADi NMPC 求解器的后端，必须在编译 CasADi 之前安装。

### 1.3 安装 OSQP

```bash
# [Jetson]
cd ~
git clone --depth 1 --branch v1.0.0 https://github.com/osqp/osqp.git
cmake -B osqp/build -S osqp -DCMAKE_BUILD_TYPE=Release
cmake --build osqp/build -j$(nproc)
sudo cmake --install osqp/build
```

### 1.4 安装 OsqpEigen

> OSQP 的 C++ 封装，`mpc_controller.cpp` 通过它调用 OSQP。

```bash
# [Jetson]
cd ~
git clone https://github.com/robotology/osqp-eigen.git
cmake -B osqp-eigen/build -S osqp-eigen -DCMAKE_BUILD_TYPE=Release
cmake --build osqp-eigen/build -j$(nproc)
sudo cmake --install osqp-eigen/build
```

### 1.5 安装 CasADi

> ⚠️ Jetson 上编译约需 10–15 分钟，请耐心等待。
> ⚠️ CasADi 安装后**不一定**导出 `casadi::casadi` CMake target，CMakeLists.txt 已处理此兼容性问题，会自动回退到库文件路径链接。

```bash
# [Jetson]
cd ~
git clone --depth 1 --branch 3.6.5 https://github.com/casadi/casadi.git
cmake -B casadi/build -S casadi \
    -DCMAKE_BUILD_TYPE=Release \
    -DWITH_IPOPT=ON -DWITH_PYTHON=OFF -DWITH_EXAMPLES=OFF
cmake --build casadi/build -j$(nproc)
sudo cmake --install casadi/build
sudo ldconfig
```

验证安装：
```bash
find /usr/local/lib -name "libcasadi*"   # 应能找到 libcasadi.so
```

---

## 二、获取源码

项目已完全自包含，硬件驱动代码位于 `driver/` 目录，**不再依赖 `pure_cpp/`**。

### 2.1 获取源码（GitHub）

```bash
# [Jetson]
cd ~
git clone https://github.com/EclipseaHime017/mpc_cpp.git
```

### 2.2 编译

```bash
# [Jetson]
cd ~/mpc_cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

编译成功后产物：

| 可执行文件 | 用途 | 需要电机 | 需要 sudo |
|-----------|------|---------|----------|
| `build/test_mpc_solver` | 离线数学验证（Step 2.4） | 否 | 否 |
| `build/test_imu` | IMU 坐标系验证（Step 5） | 否 | 否 |
| `build/test_single_joint` | 单关节方向 + 零偏标定（Step 3/4） | 是 | 是 |
| `build/mpc_robot_control` | 完整控制器（Phase A–F） | 是 | 是 |

### 2.4 离线测试（不需要硬件）

```bash
# [Jetson]
./build/test_mpc_solver config/robot_params.yaml
# 预期：4/4 passed
```

> 如果此步骤有失败，说明依赖库有问题，不要继续上机。

---

## 三、硬件逐项验证

> `[Jetson]` 所有步骤在 Jetson 上执行。
> ⚠️ **每一步通过后才进入下一步，任何步骤失败立即停止。**

### Step 1：CAN 总线通信

```bash
# [Jetson]
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000
sudo ip link set can2 up type can bitrate 1000000
sudo ip link set can3 up type can bitrate 1000000

candump can0   # 监听帧
```

**预期**：电机上电后每 1–2ms 收到一帧，ID 对应 `{1,5,9,13}`。
**排查**：无帧 → 检查终端电阻（120Ω）、波特率、接线极性。

### Step 2：IMU 串口

```bash
# [Jetson]
ls /dev/ttyCH341USB*

python3 -c "
import serial, time
s = serial.Serial('/dev/ttyCH341USB0', 115200, timeout=1)
time.sleep(0.5)
data = s.read(100)
print([hex(b) for b in data])
"
```

**预期**：输出中包含 `0x55`（WIT 协议帧头）。
**排查**：乱码 → 逐一尝试 9600 / 115200 / 230400 波特率，在 `config/robot_params.yaml` 中更新 `imu_device`。

### Step 3：单关节运动测试

> ⚠️ **最关键的安全步骤**。机器人必须固定在支架上，腿悬空。
> 使用专用工具 `test_single_joint`，内置低刚度参数（kp=10, kd=2, torque_limit=5Nm），无需手动改配置。

**用法**：

```bash
# [Jetson]
sudo ./build/test_single_joint <mpc_index>       # 测试单个关节
sudo ./build/test_single_joint all                # 读取全部 12 个电机位置
```

**mpc_index 对照表**：

| mpc_index | 关节 | CAN 总线 | Motor ID |
|-----------|------|---------|----------|
| 0 | LF_HipA | can0 | 1 |
| 1 | LF_HipF | can0 | 2 |
| 2 | LF_Knee | can0 | 3 |
| 3 | LR_HipA | can1 | 5 |
| 4 | LR_HipF | can1 | 6 |
| 5 | LR_Knee | can1 | 7 |
| 6 | RF_HipA | can2 | 9 |
| 7 | RF_HipF | can2 | 10 |
| 8 | RF_Knee | can2 | 11 |
| 9 | RR_HipA | can3 | 13 |
| 10 | RR_HipF | can3 | 14 |
| 11 | RR_Knee | can3 | 15 |

**测试流程**（每个关节独立执行）：

```bash
# 示例：测试左前髋外展
sudo ./build/test_single_joint 0
```

程序自动运行两个阶段：
1. **阶段 1（3 秒）**：静止保持，确认编码器读数稳定，打印 raw position
2. **阶段 2（10 秒）**：±0.2 rad 正弦摆动（0.3Hz），屏幕提示该关节的**预期物理方向**，肉眼确认是否一致

> Ctrl+C 随时停止，电机会缓慢归位后自动断电。

**验证结果记录表**：

| MPC 索引 | 物理关节 | 正方向含义 | 实测符合 |
|---------|---------|-----------|---------|
| 0 | LF_HipA | 左前腿外展（腿向外） | □ |
| 1 | LF_HipF | 左前腿前屈 | □ |
| 2 | LF_Knee | 左前膝弯曲 | □ |
| 3 | LR_HipA | 左后腿外展 | □ |
| 4 | LR_HipF | 左后腿前屈 | □ |
| 5 | LR_Knee | 左后膝弯曲 | □ |
| 6 | RF_HipA | 右前腿外展 | □ |
| 7 | RF_HipF | 右前腿前屈 | □ |
| 8 | RF_Knee | 右前膝弯曲 | □ |
| 9 | RR_HipA | 右后腿外展 | □ |
| 10 | RR_HipF | 右后腿前屈 | □ |
| 11 | RR_Knee | 右后膝弯曲 | □ |

**若方向反了**：在 `config/robot_params.yaml` 对应 `joint_offsets` 项取反，或在 `src/main.cpp` 的 `read_joints()` 里对该关节加负号。

### Step 4：关节零偏标定

将机器人放在标准夹具上（四腿垂直地面），使用 `test_single_joint all` 读取全部电机编码器值：

```bash
# [Jetson]
sudo ./build/test_single_joint all
```

输出示例：

```
┌─────────┬──────────────┬────────────┬────────────┬───────────────┐
│ MPC Idx │ Joint        │ Raw Pos    │ Offset     │ q_mpc         │
├─────────┼──────────────┼────────────┼────────────┼───────────────┤
│       0 │     LF_HipA  │     0.3712 │     0.3700 │        0.0012 │
│       1 │     LF_HipF  │     0.1298 │     0.1300 │       -0.0002 │
│  ...    │    ...       │    ...     │    ...     │    ...        │
└─────────┴──────────────┴────────────┴────────────┴───────────────┘

# 便于复制的 yaml 格式（当前 Raw Pos 值）：
joint_offsets:
  hipa: [0.3712, -0.3698, -0.3705, 0.3701]
  hipf: [0.1298, 0.1305, -0.1297, -0.1302]
  knee: [1.7685, 1.7678, -1.7690, -1.7675]
```

> 程序底部直接输出可复制的 yaml 格式，将 `Raw Pos` 列的值作为新的 `joint_offsets` 填入 `config/robot_params.yaml`。
> ⚠️ 此命令以**零力矩**模式读取（kp=0, kd=0, torque_limit=0），电机不会产生任何运动。

### Step 5：IMU 坐标系验证

> 使用专用工具 `test_imu`，**不涉及任何电机**，随时可安全运行。

```bash
./build/test_imu config/robot_params.yaml
```

终端实时刷新，显示三组信息：

```
--- Raw (WIT frame: X=right, Y=forward, Z=up) ---
  Quaternion [w,x,y,z]:  0.9999  0.0012  -0.0008  0.0031
  Gyro [x,y,z] rad/s:    0.001   0.000   0.001
  Accel [x,y,z] m/s²:    0.021  -0.015   9.812

--- Body frame (after R_imu2body: X=forward, Y=left, Z=up) ---
  RPY [roll,pitch,yaw] rad:      0.0011    -0.0008     1.5710
  RPY [roll,pitch,yaw] deg:      0.063     -0.046      90.0
  Omega [x,y,z] rad/s:           0.000      0.001      0.001
  Gravity dir [x,y,z]:           0.001     -0.001     -1.000

--- Sanity checks (robot flat on ground, stationary) ---
  roll  (rad):    0.0011  (expect 0.0)  ✓
  pitch (rad):   -0.0008  (expect 0.0)  ✓
  grav_x     :    0.0010  (expect 0.0)  ✓
  grav_y     :   -0.0010  (expect 0.0)  ✓
  grav_z     :   -1.0001  (expect -1.0) ✓
  omega_norm :    0.0014  (expect 0.0)  ✓
  quat norm  :    1.0000  (expect 1.0)  ✓
```

**验证方法**：

1. 将机器人平放在地面（无需上电），运行 `test_imu`
2. 确认 sanity checks 全部显示 `✓`
3. 缓慢倾斜机器人（侧倾），观察 `roll` 随之变化且方向正确（向右倾斜 → roll 为正）
4. 缓慢前后倾，观察 `pitch` 方向正确（抬头 → pitch 为正）
5. `Ctrl+C` 退出

**若 sanity check 有 `✗`**：

| 异常现象 | 原因 | 修复 |
|---------|------|------|
| roll/pitch 非零（机器人放平时） | `R_imu2body` 旋转轴对应关系错误 | 调整 `state_estimator.cpp` 中的 `R_imu2body` |
| grav_z 不为 -1 | IMU Z 轴与机体 Z 轴反向 | `R_imu2body` 第三行取反 |
| 倾斜时 roll/pitch 方向反 | 旋转符号错误 | `R_imu2body` 对应行取反 |
| quat norm ≠ 1 | IMU 未完成初始化 | 等待 1–2 秒让 IMU 收敛后重试 |

---

## 四、分阶段功能测试

> `[Jetson]` 所有测试均在 Jetson 上执行（需要 `sudo`）。

**启动流程说明**（每个 Phase 通用）：

```
sudo ./build/mpc_robot_control config/robot_params.yaml
  │
  ├─ Phase A (自动)：电机 Enable + 30步插值到站立姿态（约3s）
  │
  ├─ 打印：Press ENTER to start MPC control loop...
  │         此处 Ctrl+C 可安全中断（电机归零后断电）
  │
  └─ 按 ENTER → 进入 500Hz 控制循环
       │
       ├─ Phase B (自动 2s)：PD 保持站立 + IMU 标定
       │
       ├─ Phase C (自动)：MPC 静态站立
       │     四腿全接地，MPC 出 GRF，WBC 计算 tau_ff
       │     终端显示 "STAND" + GRF 数值
       │     此阶段可验证 MPC 输出是否正常
       │
       └─ Phase D (手柄触发)：推手柄 → 步态激活
             终端显示 "WALK"
             步态 + MPC + WBC 全部运行
```

**关键设计**：Phase C → Phase D 的切换由**手柄速度指令**触发（`|v_cmd| > 0.03 m/s` 或 `|yaw_rate| > 0.03 rad/s`），而非自动。操作员可以在 Phase C 充分验证 MPC 输出后再推手柄开始行走。

**Phase 之间的切换**：
- Phase A → B → C：程序自动推进，无需干预
- Phase C → D：**推手柄**触发，一旦激活不可回退
- 修改 yaml 参数需 `Ctrl+C` 后重新启动程序

---

### Phase A：悬空站立

**物理准备**：机器人挂在支架上，腿完全悬空。

**修改 yaml**：
```yaml
mit_kp: 30.0
mit_kd: 1.5
target_z: 0.20
```

**运行**：
```bash
sudo ./build/mpc_robot_control config/robot_params.yaml
# 等待插值完成后按 ENTER
# 观察 2s 标定阶段 + MPC 静态站立
# Ctrl+C 退出
```

**检查项**：
- [ ] 12 个关节在 3s 内缓慢运动到目标位置，无异响
- [ ] 到位后静止，无持续抖动（若抖动：降低 `mit_kp`）
- [ ] `Ctrl+C` 后电机平滑停止（归零力矩）

---

### Phase B：地面静止站立（IMU 标定）

> 此阶段在按下 ENTER 后**自动**执行约 2 秒（`calib_duration`），无需操作。

**物理准备**：机器人放到地面，**手扶机身**，保持静止。

**修改 yaml**（相对 Phase A 提高刚度）：
```yaml
mit_kp: 40.0
mit_kd: 0.5
```

**检查项**：
- [ ] 终端打印 `[Calib] Nominal foot offsets calibrated.`
- [ ] 站立高度 ≈ `target_z`（卷尺验证）

---

### Phase C：MPC 静态站立

> Phase B 结束后**自动**进入 Phase C。四腿全接地，MPC 计算 GRF，WBC 计算前馈力矩。
> **此阶段不会启动步态**，机器人保持静止，终端显示 `STAND`。

终端输出示例：
```
[  3.0s] STAND | vx=0.00 yaw=0.00 | mpc_ms=2.4 | z=0.201 | fz=[29.5,29.3,29.4,29.5]N
```

**检查项**：
- [ ] 四腿法向力之和 ≈ `mass * 9.81 = 117.7N`（允许 ±15%）
- [ ] `mpc_ms` < 5ms 且稳定（无突变）
- [ ] 四腿受力均匀（误差 < 20%）
- [ ] 轻推后能恢复（MPC 提供反力）
- [ ] 静止 30s，电机温度 < 60C

> **此处可以安全地停留任意长时间**，等到所有指标正常后再推手柄进入 Phase D。

---

### Phase D：原地踏步

> **由手柄触发**：在 Phase C 稳定后，轻推手柄摇杆。终端打印 `[Control] Gait ACTIVATED` 并切换为 `WALK` 模式。

**修改 yaml**（首次踏步用保守参数）：
```yaml
gait_period: 0.5       # 较慢步态
step_height: 0.04      # 较低抬腿
```

**操作**：
1. 启动程序 → Phase A/B/C 自动完成
2. 确认 Phase C 的 MPC GRF 正常
3. **轻推手柄**（约 0.1 m/s），机器人开始 trot
4. 松开手柄，机器人继续原地踏步（v_cmd=0 但步态已激活）

**检查项**：
- [ ] 摆腿轨迹平滑，无碰地或突变
- [ ] 步态周期与 `gait_period` 吻合（秒表验证）
- [ ] 站立腿 GRF 连续（终端无突变）

---

### Phase E：慢速直行

**修改 yaml**（在 Phase D 稳定后调整）：
```yaml
gait_period: 0.4
step_height: 0.06
```

**操作**：推手柄前进，观察直线行走。

**检查项**：
- [ ] 能稳定前进 > 3m
- [ ] 直线偏差 < 20cm/m

---

### Phase F：完整功能

```yaml
gait_period: 0.35
step_height: 0.07
```

> 当前使用 **ConvexMPC + Raibert 启发式落足点**，NMPC 暂未启用（Jetson 上求解时间 ~120ms 超出 30Hz 预算）。
> 未来若需启用 NMPC，需在 MPC 线程中恢复 `NMPCFootstepPlanner` 调用并优化求解速度。

手柄控制：

| 操作 | 动作 |
|------|------|
| 左摇杆上/下（轴1） | 前进/后退，最大 ±0.5 m/s |
| 左摇杆左/右（轴0） | 侧移，最大 ±0.5 m/s |
| 右摇杆左/右（轴3） | 偏航，最大 ±0.5 rad/s |

**检查项**：
- [ ] 转弯、侧移稳定
- [ ] ConvexMPC solve_time < 5ms
- [ ] 连续运行 > 5min 无异常

---

## 五、参数调优流程

### 5.1 MPC 权重调整

```yaml
# weights_Q: [roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz]
weights_Q: [100, 100, 200,
             50,  50, 500,
             10,  10,  20,
             20,  20,  10]
```

| 现象 | 调整方向 |
|------|---------|
| 机器人倾斜明显 | 增大 Q[0], Q[1]（roll/pitch） |
| 高度起伏大 | 增大 Q[5]（pz） |
| 速度跟踪差 | 增大 Q[9], Q[10]（vx/vy） |
| GRF 抖动/电机过热 | 增大 `weights_R` |
| MPC 求解慢 | 减小 `weights_R`，减小 `mpc_horizon` |

### 5.2 步态参数调整

| 速度范围 | `gait_period` | `step_height` |
|---------|--------------|--------------|
| < 0.2 m/s | 0.50 s | 0.04 m |
| 0.3 m/s | 0.40 s | 0.06 m |
| > 0.4 m/s | 0.35 s | 0.08 m |

| 现象 | 调整方向 |
|------|---------|
| 摆腿蹭地 | 增大 `step_height` |
| 摆腿时机身晃动 | 减小 `step_height`，增大 `duty_factor` |
| 前冲/速度超调 | 减小 Raibert 增益（`gait_generator.cpp`） |
| 转向迟钝 | 增大 Q[2]（yaw） |

### 5.3 实时性调优

```bash
# [Jetson] 锁定 CPU 最高频率
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# 以实时优先级运行
sudo chrt -f 90 ./build/mpc_robot_control config/robot_params.yaml
```

若仍有 jitter，在 `/boot/extlinux/extlinux.conf` 的 `APPEND` 行末尾加：
```
isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3
```

**性能目标**：

| 模块 | 目标 | 报警阈值 |
|------|------|---------|
| PD 循环（500Hz） | < 2ms | > 3ms |
| ConvexMPC (OSQP) | < 5ms | > 10ms |
| 电机温度 | < 60°C | > 75°C 降力矩 |

---

## 六、上机检查清单

```
【部署前 — Jetson 上】
  □ test_mpc_solver 4/4 通过
  □ joint_offsets 已用实机编码器标定
  □ IMU 坐标系验证通过（静止 rpy ≈ [0, 0, yaw]）
  □ Ctrl+C 紧急停止已测试（电机平滑断电）
  □ mit_torque_limit = 17.0（不可修改，电机硬件上限）
  □ 备有物理断电开关
  □ 手柄已连接（/dev/input/js0）

【Phase A：悬空站立】
  □ 12 个关节方向和顺序验证通过
  □ 站立姿态保持 > 30s 无抖动

【Phase B：IMU 标定】
  □ 终端打印 "Nominal foot offsets calibrated."

【Phase C：MPC 静态站立（手柄触发前）】
  □ 终端显示 STAND 模式
  □ fz_total ≈ mass * 9.81（误差 < 15%）
  □ solve_time < 5ms 稳定
  □ 四腿受力均匀（误差 < 20%）
  □ 高度与 target_z 吻合（误差 < 3cm）
  □ 轻推后能恢复

【Phase D：原地踏步（推手柄触发）】
  □ 终端显示 "Gait ACTIVATED" 并切换为 WALK 模式
  □ 摆腿平滑，连续 30 步以上无跌倒

【Phase E：慢速直行】
  □ 3m 直线行走，偏差 < 20cm

【Phase F：完整功能】
  □ 各方向稳定运动
  □ ConvexMPC solve_time < 5ms
  □ 全速行走 > 5min，电机温度正常
```

---

## 七、常见问题速查

| 现象 | 最可能原因 | 排查方法 |
|------|-----------|---------|
| `cmake` 报 `casadi::casadi not found` | CasADi 未导出 CMake target | `find /usr/local/lib -name "libcasadi*"` 确认安装，`rm -rf build` 后重新 cmake |
| `undefined reference to NMPCFootstepPlanner` | `nmpc_footstep.cpp` 未加入 sources | 已修复，`git pull` 后重新编译 |
| 电机不动 | CAN ID 不匹配 | `candump can0` 对比 `motor_ids` |
| 站立时腿方向反 | 关节符号错误 | 单关节测试，逐一对照验证表 |
| 站立时身体倾斜 | `joint_offsets` 不准 | 重新标定零偏 |
| IMU 重力方向错 | `R_imu2body` 错误 | 静止时打印 rpy，调整旋转矩阵 |
| MPC 法向力为负 | FK 足端位置误差大 | 检查 `foot_pos_world`，验证运动学 |
| 步态不启动 | 手柄未连接或未推杆 | 确认 `/dev/input/js0` 存在，推手柄超过阈值 |
| PD 循环 jitter 大 | 未开实时优先级或 CPU 降频 | `chrt -f 90`，设置 performance governor |
| 行走持续偏转 | 陀螺仪零偏未标定 | 延长 `calib_duration`，静止后再启动 |
| 摆腿蹭地 | `step_height` 过小或 FK 误差 | 增大 `step_height`，检查膝关节 FK |
