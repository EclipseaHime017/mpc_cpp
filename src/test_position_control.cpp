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

// 映射逻辑: MPC_IDX 到 MOTOR_IDX (全局硬件编号)
// LF: mpc 0-2 → gi 0,4,8   LR: mpc 3-5 → gi 1,5,9
// RF: mpc 6-8 → gi 2,6,10  RR: mpc 9-11 → gi 3,7,11
static const int MPC_TO_MOTOR_IDX[12] = {0,4,8, 1,5,9, 2,6,10, 3,7,11};

/**
 * 计算含符号的传动比
 *
 * 左腿 HipF/Knee 电机安装方向与右腿相反：
 *   pos_raw = q * gear + joint_offset
 * 左腿 gear > 0，右腿 HipF/Knee gear < 0，
 * 这样 IK 输出相同方向的 q，两侧电机均能产生正确的物理运动。
 * HipA 安装方向一致，gear = +1.0。
 */
static double get_gear(const RobotConfig& cfg, int mpc_idx) {
    const bool is_knee  = (mpc_idx % 3 == 2);
    const bool is_hipf  = (mpc_idx % 3 == 1);
    const bool is_right = (mpc_idx / 3 >= 2);  // RF=leg2, RR=leg3
    if (is_knee) return is_right ? -cfg.knee_gear_ratio : cfg.knee_gear_ratio;
    if (is_hipf && is_right) return -1.0;
    return 1.0;
}

/**
 * 摆线轨迹计算（支持 x / y 双轴步长）
 * @param t    归一化相位 [0, 1]（摆动相内）
 * @param sx   x 方向步长 (m)
 * @param sy   y 方向步长 (m)
 * @param sh   抬腿参数 (m)，实际峰值高度 = 2×sh
 * @return     相对于摆动起点的 3D 偏移 (x, y, z↓)
 */
static Eigen::Vector3d calculate_cycloid(double t, double sx, double sy, double sh) {
    auto cyc_x = [](double t_in, double len) {
        return len * (t_in - (1.0 / (2.0 * M_PI)) * std::sin(2.0 * M_PI * t_in));
    };
    double x = cyc_x(t, sx);
    double y = cyc_x(t, sy);
    double z = 0.0;
    if (t < 0.5)
        z = 2.0 * sh * (2.0 * t - (1.0 / (2.0 * M_PI)) * std::sin(4.0 * M_PI * t));
    else
        z = 2.0 * sh * (2.0 * (1.0 - t) - (1.0 / (2.0 * M_PI)) * std::sin(4.0 * M_PI * (1.0 - t)));
    return Eigen::Vector3d(x, y, z);
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    // -------------------------------------------------------------------------
    // 1. 加载配置与初始化硬件
    // -------------------------------------------------------------------------
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
            minfo->motor_id  = cfg.motor_ids[global_idx];
            minfo->host_id   = 0xFD;
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
    std::cout << "[Init] 等待系统就绪..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // -------------------------------------------------------------------------
    // 2. 平滑起立：当前位置 → 站立 (2 秒, 500 Hz)
    // -------------------------------------------------------------------------
    std::cout << "[Init] 执行起立动作..." << std::endl;

    std::vector<double> start_pos_raw(12);
    for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
        int gi        = MPC_TO_MOTOR_IDX[mpc_idx];
        start_pos_raw[mpc_idx] = rs->GetMotorState(motor_indices[gi]).position;
    }

    for (unsigned int step = 1; step <= 1000; ++step) {
        if (!g_running) break;
        float factor = (float)step / 1000.0f;

        for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
            int gi     = MPC_TO_MOTOR_IDX[mpc_idx];
            double offset = cfg.joint_offsets[gi];
            double gear   = get_gear(cfg, mpc_idx);

            // stand_joint_angles=0 → target = offset
            double target_pos_raw = cfg.stand_joint_angles[mpc_idx] * gear + offset;
            double cur = start_pos_raw[mpc_idx] * (1.0 - factor) + target_pos_raw * factor;
            {
                double lo = offset + cfg.joint_shaft_min[gi];
                double hi = offset + cfg.joint_shaft_max[gi];
                cur = std::max(lo, std::min(hi, cur));
            }
            rs->SendMITCommand(motor_indices[gi], (float)cur, 0.0f,
                               (float)cfg.kp_stand, (float)cfg.kd_stand, 0.0f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::cout << "[Init] 起立完成！" << std::endl;

    if (cfg.duty_factor <= 0.0 || cfg.duty_factor >= 1.0) {
        std::cerr << "[Error] duty_factor=" << cfg.duty_factor << " 必须在 (0,1)" << std::endl;
        return 1;
    }
    const double swing_frac  = 1.0 - cfg.duty_factor;
    const double stance_frac = cfg.duty_factor;

    double phases[4];
    for (int i = 0; i < 4; ++i) phases[i] = cfg.phase_offsets[i];

    // 足端基准位置（IK 模型坐标系，z 正向朝下）
    // h0 = (L2+L3) - z_zero：当目标高度 = z_zero 时 IK 返回 q=0
    const double h0 = (cfg.L2 + cfg.L3) - cfg.z_zero;
    Eigen::Matrix<double, 4, 3> nominal_foots;
    for (int i = 0; i < 4; ++i) {
        double s = (cfg.hip_offsets(i, 1) > 0) ? 1.0 : -1.0;
        nominal_foots(i, 0) = cfg.hip_offsets(i, 0);
        nominal_foots(i, 1) = cfg.hip_offsets(i, 1) + s * cfg.L1;
        nominal_foots(i, 2) = cfg.target_z + h0;
    }

    // -------------------------------------------------------------------------
    // 遥控器参数
    // -------------------------------------------------------------------------
    constexpr double DEADZONE   = 0.05;   // 摇杆死区（归一化，0~1）
    constexpr double MAX_VX     = 1.0;    // 前后最大速度 (m/s)
    constexpr double MAX_VY     = 0.4;    // 横向最大速度 (m/s)
    constexpr double MAX_WZ     = 0.8;    // 偏航最大速率 (rad/s)
    constexpr double HALF_TRACK = 0.135;  // 左右足横向半距 (m)，用于差速转向

    // -------------------------------------------------------------------------
    // 3. 主循环 (500 Hz)
    // -------------------------------------------------------------------------
    auto next_tick = std::chrono::steady_clock::now();
    std::cout << "[Loop] 进入控制循环" << std::endl;
    std::cout << "  左摇杆 Y=前后  X=横向平移;  右摇杆 X=偏航转向" << std::endl;
    int loop_count = 0;
    double vx = 0.0, vy = 0.0, wz = 0.0;

    while (g_running) {
        next_tick += std::chrono::microseconds(2000);
        constexpr double dt = 0.002;
        ++loop_count;

        // A. 读取遥控器，应用死区，低通滤波
        double vx_cmd = 0.0, vy_cmd = 0.0, wz_cmd = 0.0;
        if (gamepad->IsConnected()) {
            double ax0 = gamepad->GetAxis(0);  // 左摇杆 X（横向）
            double ax1 = gamepad->GetAxis(1);  // 左摇杆 Y（前后）
            double ax2 = gamepad->GetAxis(2);  // 右摇杆 X（偏航）
            if (std::abs(ax0) > DEADZONE) vy_cmd = -ax0 * MAX_VY;
            if (std::abs(ax1) > DEADZONE) vx_cmd = -ax1 * MAX_VX;
            if (std::abs(ax2) > DEADZONE) wz_cmd = -ax2 * MAX_WZ;
        }
        vx = 0.95 * vx + 0.05 * vx_cmd;
        vy = 0.95 * vy + 0.05 * vy_cmd;
        wz = 0.95 * wz + 0.05 * wz_cmd;

        // B. 判断是否处于静止死区（滤波后速度足够小）
        const bool standing = (std::abs(vx) < 0.01 &&
                                std::abs(vy) < 0.01 &&
                                std::abs(wz) < 0.01);

        if (loop_count % 500 == 0) {
            std::cout << "[DBG] vx=" << vx << " vy=" << vy << " wz=" << wz
                      << " standing=" << standing
                      << " phases=[" << phases[0] << "," << phases[1]
                      << "," << phases[2] << "," << phases[3] << "]" << std::endl;
        }

        // C. 静止模式：保持 q=0 站立姿态，不推进相位
        if (standing) {
            for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
                int gi = MPC_TO_MOTOR_IDX[mpc_idx];
                rs->SendMITCommand(motor_indices[gi],
                                   (float)cfg.joint_offsets[gi], 0.0f,
                                   (float)cfg.kp_stand, (float)cfg.kd_stand, 0.0f);
            }
            std::this_thread::sleep_until(next_tick);
            continue;
        }

        // D. 差速转向：左右腿步长不同；横向：所有腿 y 步长相同
        //    wz > 0 = 逆时针（向左转），左腿后踏，右腿前踏
        //    side: 左腿 +1，右腿 -1
        const double T_stance = cfg.gait_period * stance_frac;
        double leg_step_x[4], leg_step_y[4];
        for (int i = 0; i < 4; ++i) {
            double side = (cfg.hip_offsets(i, 1) > 0) ? 1.0 : -1.0;
            leg_step_x[i] = (vx - side * wz * HALF_TRACK) * T_stance;
            leg_step_y[i] = vy * T_stance;
        }

        // E. 逆运动学求解
        Eigen::Matrix<double, 12, 1> q_des = Eigen::Matrix<double, 12, 1>::Zero();
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
                foot_p.z() -= cyc.z();  // z 正向朝下，抬腿 = z 减小
            } else {
                double stance_t = (t - swing_frac) / stance_frac;
                foot_p = nominal_foots.row(i).transpose();
                foot_p.x() += (0.5 - stance_t) * leg_step_x[i];
                foot_p.y() += (0.5 - stance_t) * leg_step_y[i];
            }

            Eigen::Vector3d q_leg;
            if (kin.ik_foot(i, foot_p, q_leg)) {
                q_des.segment<3>(i * 3) = q_leg;
            } else {
                if (loop_count % 500 == 0)
                    std::cout << "[WARN] IK failed leg " << i
                              << " p=(" << foot_p.x() << "," << foot_p.y()
                              << "," << foot_p.z() << ")" << std::endl;
                // fallback: q=0 → pos_raw = joint_offset（站立位）
            }
        }

        // F. 发送电机指令（含符号齿比 + 绝对限位）
        for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
            int gi     = MPC_TO_MOTOR_IDX[mpc_idx];
            double offset = cfg.joint_offsets[gi];
            double gear   = get_gear(cfg, mpc_idx);

            double pos_raw = q_des[mpc_idx] * gear + offset;
            {
                double lo = offset + cfg.joint_shaft_min[gi];
                double hi = offset + cfg.joint_shaft_max[gi];
                pos_raw = std::max(lo, std::min(hi, pos_raw));
            }

            int leg_i    = mpc_idx / 3;
            bool in_swing = (phases[leg_i] < swing_frac);
            float kp = in_swing ? (float)cfg.kp_swing  : (float)cfg.kp_stance;
            float kd = in_swing ? (float)cfg.kd_swing  : (float)cfg.kd_stance;
            rs->SendMITCommand(motor_indices[gi], (float)pos_raw, 0.0f, kp, kd, 0.0f);
        }

        std::this_thread::sleep_until(next_tick);
    }

    // -------------------------------------------------------------------------
    // 4. 关机：平滑回到站立，再断电（二次 Ctrl+C 跳过结算）
    // -------------------------------------------------------------------------
    std::cout << "\n[Shutdown] 回到站立位置..." << std::endl;
    g_running = true;
    std::signal(SIGINT, signal_handler);

    for (unsigned int step = 1; step <= 500 && g_running; ++step) {
        float factor = (float)step / 500.0f;
        for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
            int gi     = MPC_TO_MOTOR_IDX[mpc_idx];
            double offset = cfg.joint_offsets[gi];
            double gear   = get_gear(cfg, mpc_idx);
            double stand  = cfg.stand_joint_angles[mpc_idx] * gear + offset;
            double cur    = rs->GetMotorState(motor_indices[gi]).position;
            double target = cur * (1.0 - factor) + stand * factor;
            {
                double lo = offset + cfg.joint_shaft_min[gi];
                double hi = offset + cfg.joint_shaft_max[gi];
                target = std::max(lo, std::min(hi, target));
            }
            rs->SendMITCommand(motor_indices[gi], (float)target, 0.0f,
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
