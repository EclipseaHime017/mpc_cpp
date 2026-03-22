# 部署与测试优化指南

> 适用平台：搭载 Robstride 电机（CAN 总线）+ WIT-Motion IMU 的四足机器人
> 对应代码：`BFYP/mpc_cpp/`

---

## 目录

1. [目标平台环境准备](#一目标平台环境准备)
2. [硬件逐项验证](#二硬件逐项验证)
3. [分阶段功能测试](#三分阶段功能测试)
4. [参数调优流程](#四参数调优流程)
5. [上机检查清单](#五上机检查清单)

---

## 一、目标平台环境准备

> **不能直接复制可执行文件**。目标平台架构（通常为 aarch64）与开发机不同，必须在目标平台上重新编译。

### 1.1 确认平台架构和 OS

```bash
uname -m          # aarch64 (Jetson/树莓派) 或 x86_64
cat /etc/os-release
gcc --version
```

### 1.2 安装系统依赖

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake git \
    libeigen3-dev libyaml-cpp-dev \
    libusb-1.0-0-dev coinor-libipopt-dev \
    can-utils iproute2
```

### 1.3 编译并安装 OSQP + CasADi

```bash
# OSQP v1.0（从源码，确保与目标架构匹配）
git clone --depth 1 --branch v1.0.0 https://github.com/osqp/osqp.git
cmake -B osqp/build -S osqp -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/usr/local
cmake --build osqp/build -j$(nproc)
sudo cmake --install osqp/build

# CasADi 3.6.5（需要 IPOPT）
wget https://github.com/casadi/casadi/releases/download/3.6.5/casadi-3.6.5.tar.gz
tar xf casadi-3.6.5.tar.gz
cmake -B casadi-3.6.5/build -S casadi-3.6.5 \
      -DCMAKE_BUILD_TYPE=Release \
      -DWITH_IPOPT=ON -DWITH_PYTHON=OFF -DWITH_EXAMPLES=OFF
cmake --build casadi-3.6.5/build -j$(nproc)
sudo cmake --install casadi-3.6.5/build
sudo ldconfig
```

### 1.4 传输源码并编译

```bash
# 在开发机上同步源码（排除编译产物）
rsync -av --exclude='build*' --exclude='build_*' \
    ~/eclipseaws/BFYP/ robot@<机器人IP>:~/BFYP/

# 在目标平台上编译
cd ~/BFYP/mpc_cpp
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)

# 先运行离线测试确认基础功能（无需硬件）
./build/test_mpc_solver config/robot_params.yaml
# 预期输出：4/4 passed
```

---

## 二、硬件逐项验证

> **每一步通过后才进入下一步，任何步骤失败立即停止。**

### Step 1：CAN 总线通信

```bash
# 启动 CAN 接口
sudo ip link set candle0 up type can bitrate 1000000
sudo ip link set candle1 up type can bitrate 1000000
sudo ip link set candle2 up type can bitrate 1000000
sudo ip link set candle3 up type can bitrate 1000000

# 监听 CAN 帧（电机上电后应持续收到帧）
candump candle0
```

**预期**：每 1–2ms 收到一帧，ID 对应电机 `{1,5,9,13}`。
**排查**：无帧 → 检查终端电阻（120Ω）、波特率、接线极性。

### Step 2：IMU 串口

```bash
# 确认设备路径
ls /dev/ttyCH341USB*

# 验证原始数据帧
python3 -c "
import serial, time
s = serial.Serial('/dev/ttyCH341USB0', 115200, timeout=1)
time.sleep(0.5)
data = s.read(100)
print([hex(b) for b in data])
"
```

**预期**：读到 WIT 协议帧头 `0x55`。
**排查**：乱码 → 逐一尝试 9600 / 115200 / 230400 波特率。

### Step 3：单关节运动测试（最关键的安全步骤）

将机器人固定在支架上，降低参数后逐个测试关节方向：

```yaml
# robot_params.yaml 临时修改
torque_limit: 5.0    # 只有 5Nm，避免暴力运动
mit_kp: 10.0
mit_kd: 2.0
```

逐一验证，填写下表：

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

**若方向反了**：在 `config/robot_params.yaml` 的对应 `joint_offsets` 项取反，
或在 `src/main.cpp` 的 `read_joints()` 里对该关节速度/位置加负号。

### Step 4：关节零偏标定

将机器人放在标准夹具上（四腿垂直于地面），读取当前电机编码器值：

```bash
# 运行简单读取程序（或在 startup 阶段临时添加打印）
# 记录各电机 raw_position 值
# joint_offset[i] = raw_position[i]（此时期望 q_mpc = 0）
```

更新 `config/robot_params.yaml`：

```yaml
joint_offsets:
  hipa: [实测值_LF, 实测值_LR, 实测值_RF, 实测值_RR]
  hipf: [实测值_LF, 实测值_LR, 实测值_RF, 实测值_RR]
  knee: [实测值_LF, 实测值_LR, 实测值_RF, 实测值_RR]
```

### Step 5：IMU 坐标系验证

```bash
sudo ./build/mpc_robot_control config/robot_params.yaml
```

机器人静止放平，观察状态估计输出：

```
[State] rpy=[ 0.001, -0.002,  1.571]   # roll≈0, pitch≈0 ✓
        pos=[ 0.000,  0.000,  0.310]   # z ≈ target_z    ✓
        omega=[0.000,  0.000,  0.000]  # 静止             ✓
```

**若 roll/pitch 不为零**：调整 `src/estimator/state_estimator.cpp` 中的 `R_imu2body` 矩阵。
**若 z 偏差 >3cm**：检查 `target_z` 设置和运动学原点定义。

---

## 三、分阶段功能测试

### Phase A：悬空站立（最安全）

机器人挂在支架上，腿完全悬空。

```yaml
mit_kp: 30.0
mit_kd: 1.5
torque_limit: 15.0
target_z: 0.20
```

```bash
sudo ./build/mpc_robot_control config/robot_params.yaml
```

**检查项**：
- [ ] 12 个关节缓慢（3s 插值）运动到目标位置，无异响
- [ ] 到位后保持静止，无持续抖动（若抖动：降低 `mit_kp`）
- [ ] `Ctrl+C` 后电机平滑停止（归零力矩）

---

### Phase B：地面静止站立

将机器人放到地面，**手扶住机身**。

```yaml
mit_kp: 40.0
mit_kd: 0.5
torque_limit: 20.0
```

**检查项**：
- [ ] 站立高度约等于 `target_z`（用卷尺验证）
- [ ] 四条腿均匀受力（可用体重秤分别称量）
- [ ] 轻推机身后能自动恢复（PD 刚度测试）
- [ ] 静止 30s 后电机温度正常（< 60°C）

---

### Phase C：MPC GRF 验证（仍静止）

开启 MPC 但保持静止，验证地面反力输出合理性。

在 `main.cpp` 中开启 MPC 输出打印，观察终端：

```
[MPC] fz=[29.5, 29.3, 29.4, 29.5]N   sum=117.7N   target=117.7N  ✓
[MPC] solve_time=2.4ms                                              ✓
```

**检查项**：
- [ ] 四腿法向力之和 ≈ `mass × 9.81`（允许 ±15%）
- [ ] 四腿受力大致均匀（不均匀说明 `foot_pos_world` 估计有误）
- [ ] `solve_time` < 5ms 且稳定

---

### Phase D：原地踏步

```yaml
gait_period: 0.5       # 先用慢步态
step_height: 0.04      # 低摆腿
ipopt_max_iter: 0      # 先关 NMPC，只用 ConvexMPC
```

手柄零输入，机器人在原地 trot。

**检查项**：
- [ ] 摆腿轨迹平滑，无碰地或突变
- [ ] 站立腿 GRF 连续（无突变）
- [ ] 步态周期与 `gait_period` 设置吻合（秒表验证）

---

### Phase E：慢速直行

```yaml
gait_period: 0.4
step_height: 0.06
ipopt_max_iter: 0      # 仍关 NMPC
```

推手柄到 50%（目标速度约 0.15 m/s）。

**检查项**：
- [ ] 能稳定前进 > 3m
- [ ] 直线偏差 < 20cm/m（在地上贴胶带做参考线）
- [ ] 无侧滑或翻倒

---

### Phase F：完整功能测试

```yaml
gait_period: 0.35
step_height: 0.07
ipopt_max_iter: 30     # 开启 NMPC 落足点优化
ipopt_tol: 0.01
```

手柄控制说明：

| 操作 | 动作 |
|------|------|
| 左摇杆上/下（轴1） | 前进/后退，最大 ±0.5 m/s |
| 左摇杆左/右（轴0） | 侧移，最大 ±0.5 m/s |
| 右摇杆左/右（轴3） | 偏航，最大 ±0.5 rad/s |

**检查项**：
- [ ] 转弯、侧移稳定
- [ ] NMPC 收敛率 > 90%（`converged` 字段）
- [ ] 连续运行 > 5min 无异常
- [ ] 各速度区间（0.1 / 0.3 / 0.5 m/s）均可稳定行走

---

## 四、参数调优流程

### 4.1 MPC 权重调整

```yaml
# weights_Q 各维度：[roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz]
weights_Q: [100, 100, 200,   # rpy
             50,  50, 500,   # pos（pz 加大 → 高度保持更好）
             10,  10,  20,   # omega
             20,  20,  10]   # vel
```

| 现象 | 调整方向 |
|------|---------|
| 机器人倾斜明显 | 增大 roll/pitch 权重（Q[0], Q[1]） |
| 高度起伏大 | 增大 pz 权重（Q[5]） |
| 速度跟踪差 | 增大 vx/vy 权重（Q[9], Q[10]） |
| GRF 抖动/电机过热 | 增大 `weights_R`（控制代价） |
| MPC 求解慢 | 减小 `weights_R`，减小 `mpc_horizon` |

### 4.2 步态参数调整

```yaml
# 速度-步态匹配参考值
# v < 0.2 m/s:  period=0.50, height=0.04
# v = 0.3 m/s:  period=0.40, height=0.06
# v > 0.4 m/s:  period=0.35, height=0.08
```

| 现象 | 调整方向 |
|------|---------|
| 摆腿时蹭地 | 增大 `step_height` |
| 摆腿时身体晃动 | 减小 `step_height`，增大 `duty_factor` |
| 前冲（速度超调） | 减小 Raibert 增益（`gait_generator.cpp`） |
| 转向迟钝 | 增大 yaw 权重 Q[2] |

### 4.3 实时性调优（嵌入式关键）

```bash
# 锁定 CPU 最高频率（防止节能降频）
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# 以实时优先级运行
sudo chrt -f 90 ./build/mpc_robot_control config/robot_params.yaml

# 若仍有 jitter（Jetson Orin 上）：隔离 CPU 核
# /boot/extlinux/extlinux.conf 的 APPEND 行末尾加：
#   isolcpus=2,3 nohz_full=2,3 rcu_nocbs=2,3
```

在 `main.cpp` 中为主循环线程绑定隔离核：

```cpp
cpu_set_t cpuset;
CPU_ZERO(&cpuset);
CPU_SET(2, &cpuset);                                     // PD 线程绑核 2
pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);
```

**性能目标**：

| 模块 | 目标 | 报警阈值 |
|------|------|---------|
| PD 循环（500Hz） | < 2ms | > 3ms |
| ConvexMPC | < 5ms | > 10ms |
| NMPC（warm-start） | < 35ms | > 60ms（降级用缓存） |
| 电机温度 | < 60°C | > 75°C 降力矩 |

### 4.4 关键监控指标

终端每 100 帧打印一次（建议在 `main.cpp` 中保留）：

```
[t=12.3s] pd=0.8ms  mpc=2.5ms  nmpc=28.1ms
          fz=118.2N  pos=(0.02, 0.00, 0.31)  rpy=(0.01, -0.01, 1.57)
```

| 指标 | 正常范围 | 异常含义 |
|------|---------|---------|
| `fz_total` | mass×9.81 ± 15% | 偏小 → 腾空或 FK 误差 |
| `pos.z` | target_z ± 0.05m | < 0.15m → 跌倒风险 |
| `rpy.roll/pitch` | < 0.15 rad | 超出 → 姿态失稳 |
| `nmpc.converged` | > 90% | 低 → 减小 horizon 或放宽 tol |

---

## 五、上机检查清单

```
【部署前】
  □ test_mpc_solver 在目标平台 4/4 通过
  □ joint_offsets 已用实机编码器标定
  □ IMU 坐标系验证通过（静止 rpy ≈ [0, 0, yaw]）
  □ Ctrl+C 紧急停止已测试（电机平滑断电）
  □ torque_limit 从低值（10Nm）开始，确认无误后逐步提高
  □ 备有手动断电开关（物理急停）

【Phase A：悬空站立】
  □ 12 个关节方向和顺序验证通过（对照验证表）
  □ 站立姿态保持 > 30s 无持续抖动

【Phase B：地面静止】
  □ 四腿受力均匀（误差 < 20%）
  □ 站立高度与 target_z 吻合（误差 < 3cm）
  □ 轻推后能自动恢复

【Phase C：MPC 静止验证】
  □ fz_total ≈ mass × 9.81（误差 < 15%）
  □ solve_time < 5ms 稳定

【Phase D：原地踏步】
  □ 摆腿平滑，高度达 step_height
  □ 连续 30 步以上无跌倒

【Phase E：慢速直行】
  □ 3m 直线行走，偏差 < 20cm
  □ NMPC 收敛率 > 90%

【Phase F：完整功能】
  □ 各方向（前/后/侧/转）稳定运动
  □ 全速（0.5 m/s）行走 > 5min 无异常
  □ 电机温度正常（< 60°C）
```

---

## 附录：常见问题速查

| 现象 | 最可能原因 | 排查方法 |
|------|-----------|---------|
| 电机不动 | CAN ID 不匹配 | 对比 `motor_ids` 与 `candump` 帧 ID |
| 站立时腿方向反 | 关节符号错误 | 单关节测试，逐一对照验证表 |
| 站立时身体倾斜 | `joint_offsets` 不准 | 重新标定零偏 |
| IMU 显示重力方向错 | `R_imu2body` 错误 | 静止时打印 rpy，调整旋转矩阵 |
| MPC 法向力为负 | FK 足端位置误差大 | 检查 `foot_pos_world` 估计，验证运动学 |
| NMPC 每次 > 50ms | IPOPT 收敛慢 | 减小 `mpc_horizon` 到 6，`ipopt_tol` 放宽到 0.05 |
| PD 循环 jitter 大 | 未开 SCHED_FIFO 或 CPU 降频 | `chrt -f 90`，设置 performance governor |
| 行走时持续偏转 | 陀螺仪零偏未标定 | 延长 `calib_duration`，静止后再启动 |
| 摆腿时蹭地 | `step_height` 过小或 FK 误差 | 增大 `step_height`，检查膝关节 FK |
