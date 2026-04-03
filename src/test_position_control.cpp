// =============================================================================
// 机器人位置控制测试 - 基于摆线轨迹与 PD 控制
// =============================================================================
#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cmath>
#include <vector>

#include "robstride.hpp"
#include "observations.hpp"
#include "common/robot_config.hpp"
#include "common/math_utils.hpp"
#include "kinematics/quadruped_kin.hpp"

// 信号处理
static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// 映射逻辑: MPC_IDX 到 MOTOR_IDX
static const int MPC_TO_MOTOR_IDX[12] = {0,4,8, 1,5,9, 2,6,10, 3,7,11};

/**
 * 摆线轨迹计算函数
 * @param t 当前相位 [0, 1]
 * @param step_len 步长 (m)
 * @param step_height 抬腿高度 (m)
 * @return 相对于起点的 3D 偏移量
 */
Eigen::Vector3d calculate_cycloid(double t, double step_len, double step_height) {
    // 摆线方程: x = L * (t - 1/(2pi) * sin(2pi*t))
    double x = step_len * (t - (1.0 / (2.0 * M_PI)) * std::sin(2.0 * M_PI * t));
    double z = 0;
    // 抬腿部分使用两个半周期的摆线
    if (t < 0.5) {
        z = 2.0 * step_height * (2.0 * t - (1.0 / (2.0 * M_PI)) * std::sin(4.0 * M_PI * t));
    } else {
        z = 2.0 * step_height * (2.0 * (1.0 - t) - (1.0 / (2.0 * M_PI)) * std::sin(4.0 * (1.0 - t)));
    }
    return Eigen::Vector3d(x, 0, z);
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);
    
    // 1. 加载配置与初始化硬件
    std::string config_path = (argc > 1) ? argv[1] : "config/robot_params.yaml";
    RobotConfig cfg = RobotConfig::from_yaml(config_path);
    QuadrupedKinematics kin(cfg);
    
    auto rs = std::make_shared<RobstrideController>();
    auto can0 = std::make_shared<CANInterface>("candle0");
    auto can1 = std::make_shared<CANInterface>("candle1");
    auto can2 = std::make_shared<CANInterface>("candle2");
    auto can3 = std::make_shared<CANInterface>("candle3");
    rs->BindCAN(can0); rs->BindCAN(can1); rs->BindCAN(can2); rs->BindCAN(can3);

    std::vector<int> motor_indices(12);
    
    std::cout << "[Init] 开始唤醒并初始化电机..." << std::endl;
    // 2. 绑定电机并稳健唤醒 (参考原 main.cpp)
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
            rs->SetMITParams(idx, {80.0f, 1.5f, 44.0f, 17.0f}); 
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    auto gamepad = std::make_shared<Gamepad>(cfg.gamepad_dev.c_str());
    std::cout << "[Init] 等待系统就绪..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 3. 初始标定与平滑起立动作 (匍匐 -> 站立)
    std::cout << "[Init] 执行起立动作 (匍匐 -> 站立)..." << std::endl;
    
    // 我们用 2 秒钟（1000个2ms的步长）来完成从 0 到目标站立角度的插值
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
            
            // 根据 factor 进行从 0 (匍匐状态) 到 target_pos_raw 的线性插值
            double current_pos_raw = target_pos_raw * factor;

            // 给定适当的 PD 参数执行平滑移动
            rs->SendMITCommand(motor_idx, (float)current_pos_raw, 0.0f, 80.0f, 1.5f, 0.0f);
        }
        // 500Hz 控制频率的休眠
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    std::cout << "[Init] 起立完成！" << std::endl;

    // 准备运动控制参数
    double gait_period = 0.5;   // 步态周期 (s)
    double step_height = 0.06;  // 抬腿高度 (m)
    
    double phases[4] = {0.0, 0.5, 0.5, 0.0}; // Trot 初始相位
    Eigen::Matrix<double, 4, 3> nominal_foots; // 初始标定足端位置
    
    // 计算站立状态下的足端基准位置
    for(int i=0; i<4; ++i) {
        Eigen::Vector3d q_leg = cfg.stand_joint_angles.segment<3>(i*3);
        nominal_foots.row(i) = kin.forward_kinematics(i, q_leg).transpose();
    }

    // 4. 主循环 (500Hz)
    auto next_tick = std::chrono::steady_clock::now();
    std::cout << "[Loop] 进入控制循环，使用左摇杆控制前进/后退速度" << std::endl;

    while (g_running) {
        next_tick += std::chrono::microseconds(2000);
        double dt = 0.002;

        // A. 获取遥控器输入
        double vx = 0.0;
        if (gamepad->IsConnected()) {
            vx = -gamepad->GetAxis(1) * 0.4; // 最大速度设定为 0.4 m/s
        }

        // B. 计算每个腿的目标
        Eigen::Matrix<double, 12, 1> q_des = Eigen::Matrix<double, 12, 1>::Zero();

        for (int i = 0; i < 4; ++i) {
            // 更新相位
            phases[i] += dt / gait_period;
            if (phases[i] > 1.0) phases[i] -= 1.0;

            double t = phases[i];
            Eigen::Vector3d foot_p;

            if (t < 0.5) { // 摆动相 (Swing)
                double swing_t = t / 0.5;
                double step_len = vx * gait_period * 0.5;
                
                // 计算相对于标定点的摆线偏移
                Eigen::Vector3d offset = calculate_cycloid(swing_t, step_len, step_height);
                
                // 摆动起点应在步长负半轴，终点在正半轴
                foot_p = nominal_foots.row(i).transpose();
                foot_p.x() += (offset.x() - step_len * 0.5);
                foot_p.z() += offset.z();
            } 
            else { // 支撑相 (Stance)
                double stance_t = (t - 0.5) / 0.5;
                double step_len = vx * gait_period * 0.5;
                
                // 支撑相足端相对于身体向后平移以推进机器人
                foot_p = nominal_foots.row(i).transpose();
                foot_p.x() += (0.5 - stance_t) * step_len;
            }

            // C. 逆运动学求解得到关节角
            Eigen::Vector3d q_leg;
            if (kin.inverse_kinematics(i, foot_p, q_leg)) {
                q_des.segment<3>(i * 3) = q_leg;
            } else {
                // IK 失败则退回到站立姿态，避免崩溃
                q_des.segment<3>(i * 3) = cfg.stand_joint_angles.segment<3>(i * 3);
            }
        }

        // D. 发送电机指令
        for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
            int gi = MPC_TO_MOTOR_IDX[mpc_idx];
            int motor_idx = motor_indices[gi];
            double offset = cfg.joint_offsets[gi];

            bool is_knee = (mpc_idx % 3 == 2);
            double gear = is_knee ? cfg.knee_gear_ratio : 1.0;
            
            // 目标角度 = 关节角 * 减速比 + 零位偏移
            double pos_raw = q_des[mpc_idx] * gear + offset;

            // MIT 模式：由于是纯位控，扭矩前馈填 0.0f
            rs->SendMITCommand(motor_idx, (float)pos_raw, 0.0f, 80.0f, 1.5f, 0.0f);
        }

        std::this_thread::sleep_until(next_tick);
    }

    // 5. 关机逻辑
    std::cout << "\n[Shutdown] 停止电机..." << std::endl;
    for (int i = 0; i < 12; ++i) {
        rs->SendMITCommand(motor_indices[i], 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        rs->DisableMotor(motor_indices[i]);
    }
    return 0;
}