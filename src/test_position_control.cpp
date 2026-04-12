// =============================================================================
// 机器人位置控制测试 - 基于摆线轨迹与 PD 控制
// =============================================================================
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include "robstride.hpp"
#include "observations.hpp"
#include "common/robot_config.hpp"
#include "kinematics/quadruped_kin.hpp"

// 信号处理
static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// 映射逻辑: MPC_IDX 到 MOTOR_IDX
static const int MPC_TO_MOTOR_IDX[12] = {0,4,8, 1,5,9, 2,6,10, 3,7,11};

/**
 * 电机方向符号
 *
 * HipF/Knee: 左腿 +1，右腿 -1（安装方向相反）
 * HipA:      棋盘格对称安装，LF/RR = +1，LR/RF = -1
 *            （前后腿 HipA 电机朝向相反，因此需要交叉取符号）
 *
 * 对正/反向行走不影响（行走时 delta_q_HipA ≈ 0）；
 * 横向平移时 delta_q_HipA ≠ 0，需要正确符号才能同向侧移。
 */
static double motor_dir(int leg_i, int joint_type) {
    if (joint_type == 1 || joint_type == 2) {        // HipF / Knee
        return (leg_i >= 2) ? -1.0 : 1.0;            // RF/RR = -1, LF/LR = +1
    }
    // HipA: LF(0)/RR(3) = +1,  LR(1)/RF(2) = -1
    return (leg_i == 0 || leg_i == 3) ? 1.0 : -1.0;
}

/**
 * 摆线轨迹（双轴：x 前后，y 横向，z 抬腿）
 * 峰值高度 = step_height（z 最大值）
 */
static Eigen::Vector3d calculate_cycloid(double t,
                                          double sx, double sy,
                                          double step_height) {
    auto cyc = [](double t_in, double len) {
        return len * (t_in - (1.0 / (2.0 * M_PI)) * std::sin(2.0 * M_PI * t_in));
    };
    double z = 0.0;
    if (t < 0.5)
        z = step_height * (2.0 * t - (1.0 / (2.0 * M_PI)) * std::sin(4.0 * M_PI * t));
    else
        z = step_height * (2.0 * (1.0 - t) - (1.0 / (2.0 * M_PI)) * std::sin(4.0 * M_PI * (1.0 - t)));
    return Eigen::Vector3d(cyc(t, sx), cyc(t, sy), z);
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    // 1. 加载配置与初始化硬件
    std::string config_path = (argc > 1) ? argv[1] : "config/robot_params.yaml";
    RobotConfig cfg = RobotConfig::from_yaml(config_path);
    QuadrupedKinematics kin(cfg);

    auto rs = std::make_shared<RobstrideController>();
    if (cfg.can_interfaces.size() < 4) {
        std::cerr << "[Error] 需要 4 个 CAN 接口，配置中只有 "
                  << cfg.can_interfaces.size() << " 个" << std::endl;
        return 1;
    }
    std::vector<std::shared_ptr<CANInterface>> can_ifaces;
    for (int j = 0; j < 4; ++j) {
        auto iface = std::make_shared<CANInterface>(cfg.can_interfaces[j].c_str());
        rs->BindCAN(iface);
        can_ifaces.push_back(iface);
    }

    std::vector<int> motor_indices(12);

    std::cout << "[Init] 开始唤醒并初始化电机..." << std::endl;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 3; ++i) {
            int global_idx = j + i * 4;
            auto minfo = std::make_unique<RobstrideController::MotorInfo>();
            minfo->motor_id = cfg.motor_ids[global_idx];
            minfo->host_id = 0xFD;
            int idx = rs->BindMotor(cfg.can_interfaces[j].c_str(), std::move(minfo));
            motor_indices[global_idx] = idx;

            rs->EnableMotor(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            rs->EnableAutoReport(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            rs->EnableAutoReport(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            rs->SetMITParams(idx, {(float)cfg.mit_kp, (float)cfg.mit_kd,
                                   (float)cfg.mit_vel_limit, (float)cfg.mit_torque_limit});
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    auto gamepad = std::make_shared<Gamepad>(cfg.gamepad_dev.c_str());
    // 等待所有电机回传第一帧反馈，最多 5 秒
    std::cout << "[Init] 等待系统就绪..." << std::endl;
    {
        auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
        while (g_running && std::chrono::steady_clock::now() < deadline) {
            bool all_online = true;
            for (int i = 0; i < 12; ++i) {
                if (!rs->IsMotorOnline(motor_indices[i])) { all_online = false; break; }
            }
            if (all_online) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        // 检查是否真的全部上线
        int online_count = 0;
        for (int i = 0; i < 12; ++i)
            if (rs->IsMotorOnline(motor_indices[i])) ++online_count;
        if (online_count < 12)
            std::cerr << "[Init] 警告: 只有 " << online_count << "/12 个电机上线\n";
    }

    // 2. 平滑起立 (当前位置 → 站立, 2 秒)
    std::cout << "[Init] 执行起立动作..." << std::endl;

    std::vector<double> start_pos_raw(12);
    for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
        int gi = MPC_TO_MOTOR_IDX[mpc_idx];
        start_pos_raw[mpc_idx] = rs->GetMotorState(motor_indices[gi]).position;
    }
    for (unsigned int step = 1; step <= 1000; ++step) {
        if (!g_running) break;
        float factor = (float)step / 1000.0f;
        for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
            int gi     = MPC_TO_MOTOR_IDX[mpc_idx];
            double off = cfg.joint_offsets[gi];
            bool is_knee = (mpc_idx % 3 == 2);
            double gear  = is_knee ? cfg.knee_gear_ratio : 1.0;
            double target = cfg.stand_joint_angles[mpc_idx] * gear + off;
            double cur = start_pos_raw[mpc_idx] * (1.0 - factor) + target * factor;
            cur = std::max(off + cfg.joint_shaft_min[gi],
                  std::min(off + cfg.joint_shaft_max[gi], cur));
            rs->SendMITCommand(motor_indices[gi], (float)cur, 0.0f,
                               (float)cfg.kp_stand, (float)cfg.kd_stand, 0.0f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::cout << "[Init] 起立完成！" << std::endl;

    if (cfg.duty_factor <= 0.0 || cfg.duty_factor >= 1.0) {
        std::cerr << "[Error] duty_factor 必须在 (0,1)" << std::endl;
        return 1;
    }
    const double swing_frac  = 1.0 - cfg.duty_factor;
    const double stance_frac = cfg.duty_factor;

    double phases[4];
    for (int i = 0; i < 4; ++i) phases[i] = cfg.phase_offsets[i];

    Eigen::Matrix<double, 4, 3> nominal_foots;
    for (int i = 0; i < 4; ++i) {
        double s = (cfg.hip_offsets(i, 1) > 0) ? 1.0 : -1.0;
        nominal_foots(i, 0) = cfg.hip_offsets(i, 0);
        nominal_foots(i, 1) = cfg.hip_offsets(i, 1) + s * cfg.L1;
        nominal_foots(i, 2) = cfg.target_z;
    }

    // 预计算站立 IK 角度（用于 delta_q 方式驱动电机）
    Eigen::Matrix<double, 12, 1> q_stand = Eigen::Matrix<double, 12, 1>::Zero();
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d q_leg;
        if (!kin.ik_foot(i, nominal_foots.row(i).transpose(), q_leg)) {
            std::cerr << "[Error] 站立 IK 失败，腿 " << i << std::endl;
            return 1;
        }
        q_stand.segment<3>(i * 3) = q_leg;
    }
    std::cout << "[Init] 站立 IK 完成" << std::endl;

    // -------------------------------------------------------------------------
    // 遥控器轴映射（Xbox 360 / XInput 布局）：
    //   axis 0 = 左摇杆 X   axis 1 = 左摇杆 Y
    //   axis 3 = 右摇杆 X   axis 4 = 右摇杆 Y
    // PS4/DualShock：右摇杆 X = axis 2，需改下方 AX_YAW
    constexpr int  AX_LATERAL = 0;   // 左摇杆 X → 横向平移
    constexpr int  AX_FWD     = 1;   // 左摇杆 Y → 前后
    constexpr int  AX_YAW     = 3;   // 右摇杆 X → 偏航（Xbox=3, PS4=2）

    constexpr double DEADZONE    = 0.05;   // 摇杆死区（归一化轴值）
    constexpr double MAX_VX      = 1.0;    // 前后最大速度 (m/s)
    constexpr double MAX_VY      = 0.4;    // 横向最大速度 (m/s)
    constexpr double MAX_WZ      = 0.8;    // 偏航最大速率 (rad/s)
    constexpr double HALF_TRACK  = 0.135;  // 左右足横向半距 (m)

    // 静止模式需要连续 N 帧检测到零输入，防止单帧噪声触发冻结
    constexpr int   STAND_HYSTERESIS = 50; // 50 帧 = 0.1 s
    int stand_count = 0;       // 连续零输入帧数
    bool in_stand_mode = false;

    // -------------------------------------------------------------------------
    // 3. 主循环 (500 Hz)
    // -------------------------------------------------------------------------
    auto next_tick = std::chrono::steady_clock::now();
    std::cout << "[Loop] 进入控制循环" << std::endl;
    std::cout << "  左 Y=前后  左 X=横向平移  右 X=偏航转向  死区=" << DEADZONE << std::endl;

    int loop_count = 0;
    double vx = 0.0, vy = 0.0, wz = 0.0;

    while (g_running) {
        next_tick += std::chrono::microseconds(2000);

        // 防止计时器积累过多欠时（例如系统负载高时），避免死循环空转
        {
            auto now = std::chrono::steady_clock::now();
            if (next_tick < now - std::chrono::milliseconds(20))
                next_tick = now;
        }

        ++loop_count;
        const double dt = 0.002;

        // A. 读取遥控器，应用死区（直接使用，不做低通滤波）
        //    只有手柄真正连接时才读取输入；断连则输入清零（不进入静止模式）
        bool gamepad_connected = gamepad->IsConnected();
        vx = 0.0; vy = 0.0; wz = 0.0;
        if (gamepad_connected) {
            double ax_fwd = gamepad->GetAxis(AX_FWD);
            double ax_lat = gamepad->GetAxis(AX_LATERAL);
            double ax_yaw = gamepad->GetAxis(AX_YAW);
            if (std::abs(ax_fwd) > DEADZONE) vx = -ax_fwd * MAX_VX;
            if (std::abs(ax_lat) > DEADZONE) vy = -ax_lat * MAX_VY;
            if (std::abs(ax_yaw) > DEADZONE) wz = -ax_yaw * MAX_WZ;
        }

        // B. 静止模式判断（迟滞：需连续 STAND_HYSTERESIS 帧零输入）
        //    gamepad 断连时不进入静止模式，避免意外冻结
        bool all_zero = (vx == 0.0 && vy == 0.0 && wz == 0.0 && gamepad_connected);
        if (all_zero) {
            if (stand_count < STAND_HYSTERESIS) ++stand_count;
        } else {
            stand_count = 0;
        }
        in_stand_mode = (stand_count >= STAND_HYSTERESIS);

        if (loop_count % 500 == 0) {
            std::cout << "[DBG] vx=" << vx << " vy=" << vy << " wz=" << wz
                      << " stand=" << in_stand_mode
                      << " phases=[" << phases[0] << "," << phases[1]
                      << "," << phases[2] << "," << phases[3] << "]" << std::endl;
        }

        // C. 静止模式：保持 offset 位置，相位不推进
        if (in_stand_mode) {
            for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
                int gi = MPC_TO_MOTOR_IDX[mpc_idx];
                rs->SendMITCommand(motor_indices[gi],
                                   (float)cfg.joint_offsets[gi], 0.0f,
                                   (float)cfg.kp_stand, (float)cfg.kd_stand, 0.0f);
            }
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        // D. 差速转向 + 横向步长
        //    wz > 0 = 逆时针（向左转）：左腿步长减小，右腿步长增大
        //    vy > 0 = 向左平移（+y 方向为左侧）
        const double T_stance = cfg.gait_period * stance_frac;
        double leg_step_x[4], leg_step_y[4];
        for (int i = 0; i < 4; ++i) {
            double side = (cfg.hip_offsets(i, 1) > 0) ? 1.0 : -1.0;  // 左=+1, 右=-1
            leg_step_x[i] = (vx - side * wz * HALF_TRACK) * T_stance;
            leg_step_y[i] = vy * T_stance;
        }

        // E. IK 求解
        Eigen::Matrix<double, 12, 1> q_des = q_stand;  // 默认保持站立角度
        for (int i = 0; i < 4; ++i) {
            phases[i] += dt / cfg.gait_period;
            if (phases[i] > 1.0) phases[i] -= 1.0;

            double t = phases[i];
            Eigen::Vector3d foot_p;

            if (t < swing_frac) {
                double swing_t = t / swing_frac;
                Eigen::Vector3d cyc = calculate_cycloid(swing_t,
                                                         leg_step_x[i], leg_step_y[i],
                                                         cfg.step_height);
                foot_p = nominal_foots.row(i).transpose();
                foot_p.x() += cyc.x() - leg_step_x[i] * 0.5;
                foot_p.y() += cyc.y() - leg_step_y[i] * 0.5;
                foot_p.z() -= cyc.z();
            } else {
                double stance_t = (t - swing_frac) / stance_frac;
                foot_p = nominal_foots.row(i).transpose();
                foot_p.x() += (0.5 - stance_t) * leg_step_x[i];
                foot_p.y() += (0.5 - stance_t) * leg_step_y[i];
            }

            Eigen::Vector3d q_leg;
            if (kin.ik_foot(i, foot_p, q_leg)) {
                q_des.segment<3>(i * 3) = q_leg;
            } else if (loop_count % 500 == 0) {
                std::cout << "[WARN] IK failed leg " << i
                          << " p=(" << foot_p.x() << "," << foot_p.y()
                          << "," << foot_p.z() << ")" << std::endl;
            }
        }

        // F. 发送电机指令
        for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
            int gi         = MPC_TO_MOTOR_IDX[mpc_idx];
            double off     = cfg.joint_offsets[gi];
            int leg_i      = mpc_idx / 3;
            int joint_type = mpc_idx % 3;
            bool is_knee   = (joint_type == 2);
            double gear    = is_knee ? cfg.knee_gear_ratio : 1.0;
            double mdir    = motor_dir(leg_i, joint_type);

            double delta_q = q_des[mpc_idx] - q_stand[mpc_idx];
            double pos_raw = delta_q * gear * mdir + off;
            {
                double lo = off + cfg.joint_shaft_min[gi];
                double hi = off + cfg.joint_shaft_max[gi];
                if ((pos_raw < lo || pos_raw > hi) && loop_count % 100 == 0)
                    std::cout << "[WARN] L" << leg_i << "J" << joint_type
                              << " 超限 pos=" << pos_raw
                              << " [" << lo << "," << hi << "]" << std::endl;
                pos_raw = std::max(lo, std::min(hi, pos_raw));
            }

            bool in_swing = (phases[leg_i] < swing_frac);
            float kp = in_swing ? (float)cfg.kp_swing  : (float)cfg.kp_stance;
            float kd = in_swing ? (float)cfg.kd_swing  : (float)cfg.kd_stance;
            rs->SendMITCommand(motor_indices[gi], (float)pos_raw, 0.0f, kp, kd, 0.0f);
        }

        std::this_thread::sleep_until(next_tick);
    }

    // 4. 关机：平滑回到站立，再断电（二次 Ctrl+C 跳过）
    std::cout << "\n[Shutdown] 回到站立位置..." << std::endl;
    g_running = true;
    std::signal(SIGINT, signal_handler);
    for (unsigned int step = 1; step <= 500 && g_running; ++step) {
        float factor = (float)step / 500.0f;
        for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
            int gi   = MPC_TO_MOTOR_IDX[mpc_idx];
            double off = cfg.joint_offsets[gi];
            bool is_knee = (mpc_idx % 3 == 2);
            double gear  = is_knee ? cfg.knee_gear_ratio : 1.0;
            double stand = cfg.stand_joint_angles[mpc_idx] * gear + off;
            double cur   = rs->GetMotorState(motor_indices[gi]).position;
            double tgt   = cur * (1.0 - factor) + stand * factor;
            tgt = std::max(off + cfg.joint_shaft_min[gi],
                  std::min(off + cfg.joint_shaft_max[gi], tgt));
            rs->SendMITCommand(motor_indices[gi], (float)tgt, 0.0f,
                               (float)cfg.kp_stand, (float)cfg.kd_stand, 0.0f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::cout << "[Shutdown] 关闭电机..." << std::endl;
    for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx)
        rs->DisableMotor(motor_indices[MPC_TO_MOTOR_IDX[mpc_idx]]);
    std::cout << "[Shutdown] 完成。" << std::endl;
    return 0;
}
