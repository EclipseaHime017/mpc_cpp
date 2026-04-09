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
 * 摆线轨迹计算函数（支持 x / y 双轴步长）
 * @param t          归一化相位 [0, 1]（摆动相内）
 * @param step_len   x 方向步长 (m)
 * @param step_len_y y 方向步长 (m)
 * @param step_height 抬腿高度 (m)，实际峰值 = step_height（z 峰值）
 */
static Eigen::Vector3d calculate_cycloid(double t,
                                          double step_len,
                                          double step_len_y,
                                          double step_height) {
    // 摆线方程: x = L * (t - 1/(2pi) * sin(2pi*t))
    auto cyc = [](double t_in, double len) {
        return len * (t_in - (1.0 / (2.0 * M_PI)) * std::sin(2.0 * M_PI * t_in));
    };
    double x = cyc(t, step_len);
    double y = cyc(t, step_len_y);
    double z = 0.0;
    if (t < 0.5)
        z = step_height * (2.0 * t - (1.0 / (2.0 * M_PI)) * std::sin(4.0 * M_PI * t));
    else
        z = step_height * (2.0 * (1.0 - t) - (1.0 / (2.0 * M_PI)) * std::sin(4.0 * M_PI * (1.0 - t)));
    return Eigen::Vector3d(x, y, z);
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
    // 2. 绑定电机并稳健唤醒
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 3; ++i) {
            int global_idx = j + i * 4;
            auto minfo = std::make_unique<RobstrideController::MotorInfo>();
            minfo->motor_id = cfg.motor_ids[global_idx];
            minfo->host_id = 0xFD;
            int idx = rs->BindMotor(cfg.can_interfaces[j].c_str(), std::move(minfo));
            motor_indices[global_idx] = idx;

            // 启用电机与自动反馈，加入延时确保 CAN 总线不丢包
            rs->EnableMotor(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            rs->EnableAutoReport(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            rs->EnableAutoReport(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));

            // 设置默认的 MIT 模式参数和限幅
            rs->SetMITParams(idx, {(float)cfg.mit_kp, (float)cfg.mit_kd,
                                   (float)cfg.mit_vel_limit, (float)cfg.mit_torque_limit});
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    auto gamepad = std::make_shared<Gamepad>(cfg.gamepad_dev.c_str());
    std::cout << "[Init] 等待系统就绪..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 3. 初始标定与平滑起立动作 (当前位置 -> 站立)
    std::cout << "[Init] 执行起立动作 (当前位置 -> 站立)..." << std::endl;

    // 读取各关节当前位置作为插值起点，避免第一帧突变
    std::vector<double> start_pos_raw(12);
    for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
        int gi = MPC_TO_MOTOR_IDX[mpc_idx];
        int motor_idx = motor_indices[gi];
        start_pos_raw[mpc_idx] = rs->GetMotorState(motor_idx).position;
    }

    // 用 2 秒钟（1000个2ms的步长）从当前位置插值到目标站立角度
    unsigned int startup_interval = 1000;
    for (unsigned int step = 1; step <= startup_interval; ++step) {
        if (!g_running) break;
        float factor = (float)step / (float)startup_interval;

        for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
            int gi = MPC_TO_MOTOR_IDX[mpc_idx];
            int motor_idx = motor_indices[gi];
            double offset = cfg.joint_offsets[gi];

            bool is_knee = (mpc_idx % 3 == 2);
            double gear = is_knee ? cfg.knee_gear_ratio : 1.0;

            // 目标站立角度的物理绝对位置
            double target_pos_raw = cfg.stand_joint_angles[mpc_idx] * gear + offset;

            // 从实际当前位置插值到目标，避免第一帧跳变
            double current_pos_raw = start_pos_raw[mpc_idx] * (1.0 - factor) + target_pos_raw * factor;
            {
                double lower = offset + cfg.joint_shaft_min[gi];
                double upper = offset + cfg.joint_shaft_max[gi];
                current_pos_raw = std::max(lower, std::min(upper, current_pos_raw));
            }

            rs->SendMITCommand(motor_idx, (float)current_pos_raw, 0.0f,
                               (float)cfg.kp_stand, (float)cfg.kd_stand, 0.0f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::cout << "[Init] 起立完成！" << std::endl;

    if (cfg.duty_factor <= 0.0 || cfg.duty_factor >= 1.0) {
        std::cerr << "[Error] duty_factor=" << cfg.duty_factor
                  << " 必须在 (0, 1) 之间" << std::endl;
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
        // 直接让 IK 在真实的 target_z 处求解
        nominal_foots(i, 2) = cfg.target_z;
    }

    // 预计算站立 IK 角度，用于 delta_q 方式驱动电机
    Eigen::Matrix<double, 12, 1> q_stand = Eigen::Matrix<double, 12, 1>::Zero();
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d q_leg;
        if (kin.ik_foot(i, nominal_foots.row(i).transpose(), q_leg)) {
            q_stand.segment<3>(i * 3) = q_leg;
        } else {
            std::cerr << "[Error] 站立 IK 失败，腿 " << i << " 无法求解" << std::endl;
            return 1;
        }
    }
    std::cout << "[Init] 计算出站立 IK 角度完成" << std::endl;

    // -------------------------------------------------------------------------
    // 遥控器参数
    // -------------------------------------------------------------------------
    constexpr double DEADZONE   = 0.05;   // 摇杆死区（归一化，0~1）
    constexpr double MAX_VX     = 1.0;    // 前后最大速度 (m/s)
    constexpr double MAX_VY     = 0.4;    // 横向最大速度 (m/s)
    constexpr double MAX_WZ     = 0.8;    // 偏航最大速率 (rad/s)
    constexpr double HALF_TRACK = 0.135;  // 左右足横向半距 (m)，用于差速转向

    // 4. 主循环 (500Hz)
    auto next_tick = std::chrono::steady_clock::now();
    std::cout << "[Loop] 进入控制循环" << std::endl;
    std::cout << "  左摇杆 Y=前后  X=横向平移;  右摇杆 X=偏航转向" << std::endl;
    int loop_count = 0;
    double vx = 0.0, vy = 0.0, wz = 0.0;

    while (g_running) {
        next_tick += std::chrono::microseconds(2000);
        double dt = 0.002;
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

        // 判断是否处于静止死区（滤波后速度足够小）
        const bool standing = (std::abs(vx) < 0.01 &&
                                std::abs(vy) < 0.01 &&
                                std::abs(wz) < 0.01);

        if (loop_count % 500 == 0) {
            std::cout << "[DBG] vx=" << vx << " vy=" << vy << " wz=" << wz
                      << " standing=" << standing
                      << " phases=[" << phases[0] << "," << phases[1]
                      << "," << phases[2] << "," << phases[3] << "]" << std::endl;
        }

        // B. 静止模式：保持站立姿态，发送 offset，不推进相位
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

        // C. 差速转向：左右腿 x 步长不同；横向：所有腿 y 步长相同
        //    wz > 0 = 逆时针（向左转），左腿后踏，右腿前踏
        //    side: 左腿 +1，右腿 -1
        const double T_stance = cfg.gait_period * stance_frac;
        double leg_step_x[4], leg_step_y[4];
        for (int i = 0; i < 4; ++i) {
            double side = (cfg.hip_offsets(i, 1) > 0) ? 1.0 : -1.0;
            leg_step_x[i] = (vx - side * wz * HALF_TRACK) * T_stance;
            leg_step_y[i] = vy * T_stance;
        }

        // D. 计算每个腿的目标关节角
        Eigen::Matrix<double, 12, 1> q_des = Eigen::Matrix<double, 12, 1>::Zero();

        for (int i = 0; i < 4; ++i) {
            // 更新相位
            phases[i] += dt / cfg.gait_period;
            if (phases[i] > 1.0) phases[i] -= 1.0;

            double t = phases[i];
            Eigen::Vector3d foot_p;

            if (t < swing_frac) { // 摆动相 (Swing)
                double swing_t = t / swing_frac;
                Eigen::Vector3d cyc = calculate_cycloid(swing_t,
                                                         leg_step_x[i], leg_step_y[i],
                                                         cfg.step_height);
                foot_p = nominal_foots.row(i).transpose();
                foot_p.x() += cyc.x() - leg_step_x[i] * 0.5;
                foot_p.y() += cyc.y() - leg_step_y[i] * 0.5;
                foot_p.z() -= cyc.z();  // z 正方向朝下，抬脚 = z 减小
            }
            else { // 支撑相 (Stance)
                double stance_t = (t - swing_frac) / stance_frac;
                foot_p = nominal_foots.row(i).transpose();
                foot_p.x() += (0.5 - stance_t) * leg_step_x[i];
                foot_p.y() += (0.5 - stance_t) * leg_step_y[i];
            }

            // 逆运动学求解
            Eigen::Vector3d q_leg;
            if (kin.ik_foot(i, foot_p, q_leg)) {
                q_des.segment<3>(i * 3) = q_leg;
            } else {
                if (loop_count % 500 == 0)
                    std::cout << "[WARN] IK failed leg " << i
                              << " foot_p=(" << foot_p.x() << "," << foot_p.y()
                              << "," << foot_p.z() << ")" << std::endl;
                q_des.segment<3>(i * 3) = q_stand.segment<3>(i * 3);  // fallback: 保持站立
            }
        }

        // E. 发送电机指令
        for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
            int gi = MPC_TO_MOTOR_IDX[mpc_idx];
            int motor_idx = motor_indices[gi];
            double offset = cfg.joint_offsets[gi];

            bool is_knee = (mpc_idx % 3 == 2);
            double gear = is_knee ? cfg.knee_gear_ratio : 1.0;

            // 右侧腿 (RF, RR) 的 HipF/Knee 电机安装方向与左侧相反
            int leg_i = mpc_idx / 3;
            int joint_type = mpc_idx % 3;
            double motor_dir = ((leg_i == 2 || leg_i == 3) && (joint_type == 1 || joint_type == 2))
                               ? -1.0 : 1.0;

            double delta_q = q_des[mpc_idx] - q_stand[mpc_idx];
            double pos_raw = delta_q * gear * motor_dir + offset;
            {
                double lower = offset + cfg.joint_shaft_min[gi];
                double upper = offset + cfg.joint_shaft_max[gi];
                if (pos_raw < lower || pos_raw > upper) {
                    if (loop_count % 100 == 0)
                        std::cout << "[WARN] Leg " << leg_i << " Joint " << joint_type
                                  << " 超限! pos_raw=" << pos_raw
                                  << " 限位=[" << lower << "," << upper << "]" << std::endl;
                }
                pos_raw = std::max(lower, std::min(upper, pos_raw));
            }

            bool in_swing = (phases[leg_i] < swing_frac);
            float kp = in_swing ? (float)cfg.kp_swing  : (float)cfg.kp_stance;
            float kd = in_swing ? (float)cfg.kd_swing  : (float)cfg.kd_stance;
            rs->SendMITCommand(motor_idx, (float)pos_raw, 0.0f, kp, kd, 0.0f);
        }

        std::this_thread::sleep_until(next_tick);
    }

    // 5. 关机：先让腿回到站立位置，再关闭电机（再次 Ctrl+C 可跳过结算直接断电）
    std::cout << "\n[Shutdown] 回到站立位置..." << std::endl;
    g_running = true;
    std::signal(SIGINT, signal_handler);
    for (unsigned int step = 1; step <= 500 && g_running; ++step) {
        float factor = (float)step / 500.0f;
        for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
            int gi = MPC_TO_MOTOR_IDX[mpc_idx];
            int motor_idx = motor_indices[gi];
            double offset = cfg.joint_offsets[gi];
            bool is_knee = (mpc_idx % 3 == 2);
            double gear = is_knee ? cfg.knee_gear_ratio : 1.0;
            double stand_pos_raw = cfg.stand_joint_angles[mpc_idx] * gear + offset;
            double cur = rs->GetMotorState(motor_idx).position;
            double target = cur * (1.0 - factor) + stand_pos_raw * factor;
            {
                double lower = offset + cfg.joint_shaft_min[gi];
                double upper = offset + cfg.joint_shaft_max[gi];
                target = std::max(lower, std::min(upper, target));
            }
            rs->SendMITCommand(motor_idx, (float)target, 0.0f,
                               (float)cfg.kp_stand, (float)cfg.kd_stand, 0.0f);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::cout << "[Shutdown] 关闭电机..." << std::endl;
    for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
        int gi = MPC_TO_MOTOR_IDX[mpc_idx];
        rs->DisableMotor(motor_indices[gi]);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::cout << "[Shutdown] 完成。" << std::endl;
    return 0;
}
