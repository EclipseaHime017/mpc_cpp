// =============================================================================
// Single Joint Test Tool
// =============================================================================
// 逐个测试电机方向、零偏、通信是否正常。
//
// 用法：
//   sudo ./build/test_single_joint <mpc_index> [config_path]
//
// mpc_index 含义：
//   0=LF_HipA  1=LF_HipF  2=LF_Knee
//   3=LR_HipA  4=LR_HipF  5=LR_Knee
//   6=RF_HipA  7=RF_HipF  8=RF_Knee
//   9=RR_HipA 10=RR_HipF 11=RR_Knee
//
// 功能：
//   1. 初始化对应 CAN 总线和单个电机
//   2. 读取当前位置（raw encoder），打印用于标定 joint_offset
//   3. 缓慢发送 +0.2 rad 正弦命令，观察物理方向
//   4. Ctrl+C 停止，电机归零
// =============================================================================

#include <iostream>
#include <iomanip>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <csignal>
#include <atomic>

#include "robstride.hpp"
#include "can_interface.hpp"
#include "common/robot_config.hpp"

static std::atomic<bool> g_running{true};
static void sig_handler(int) { g_running = false; }

// Motor layout (same as main.cpp)
// motor_indices order: [LF_HipA,LR_HipA,RF_HipA,RR_HipA,
//                       LF_HipF,LR_HipF,RF_HipF,RR_HipF,
//                       LF_Knee,LR_Knee,RF_Knee,RR_Knee]
// CAN bus:  candle0=LF, candle1=LR, candle2=RF, candle3=RR
// Motor ID: HipA=1/5/9/13, HipF=2/6/10/14, Knee=3/7/11/15

// MPC index -> (CAN bus index, Motor ID, joint name, joint_offset index)
struct JointInfo {
    int can_bus;      // 0-3 -> candle0-candle3
    int motor_id;
    int offset_idx;   // index in cfg.joint_offsets[12]
    const char* name;
    bool is_knee;
};

// MPC order: [LF_HipA, LF_HipF, LF_Knee, LR_..., RF_..., RR_...]
// motor_indices order: [LF_HipA(0), LR_HipA(1), RF_HipA(2), RR_HipA(3),
//                       LF_HipF(4), LR_HipF(5), RF_HipF(6), RR_HipF(7),
//                       LF_Knee(8), LR_Knee(9), RF_Knee(10), RR_Knee(11)]
static const int MPC_TO_MOTOR_IDX[12] = {0,4,8, 1,5,9, 2,6,10, 3,7,11};

static const JointInfo JOINTS[12] = {
    // MPC 0-2: LF leg -> candle0
    {0,  1, 0,  "LF_HipA", false},   // MPC 0
    {0,  2, 4,  "LF_HipF", false},   // MPC 1
    {0,  3, 8,  "LF_Knee", true },   // MPC 2
    // MPC 3-5: LR leg -> candle1
    {1,  5, 1,  "LR_HipA", false},   // MPC 3
    {1,  6, 5,  "LR_HipF", false},   // MPC 4
    {1,  7, 9,  "LR_Knee", true },   // MPC 5
    // MPC 6-8: RF leg -> candle2
    {2,  9, 2,  "RF_HipA", false},   // MPC 6
    {2, 10, 6,  "RF_HipF", false},   // MPC 7
    {2, 11, 10, "RF_Knee", true },   // MPC 8
    // MPC 9-11: RR leg -> candle3
    {3, 13, 3,  "RR_HipA", false},   // MPC 9
    {3, 14, 7,  "RR_HipF", false},   // MPC 10
    {3, 15, 11, "RR_Knee", true },   // MPC 11
};

static void print_usage() {
    std::cout << "用法: sudo ./test_single_joint <mpc_index> [config.yaml]\n\n"
              << "mpc_index:\n"
              << "  0=LF_HipA  1=LF_HipF  2=LF_Knee\n"
              << "  3=LR_HipA  4=LR_HipF  5=LR_Knee\n"
              << "  6=RF_HipA  7=RF_HipF  8=RF_Knee\n"
              << "  9=RR_HipA 10=RR_HipF 11=RR_Knee\n\n"
              << "  all  — 依次读取全部 12 个电机当前位置（用于标定 joint_offset）\n";
}

// 模式 1: 读取全部 12 个电机的当前位置
static int read_all_positions(const RobotConfig& cfg) {
    std::cout << "\n===== 读取全部 12 个电机当前位置 =====\n\n";

    auto rs = std::make_shared<RobstrideController>();
    std::shared_ptr<CANInterface> cans[4];
    const char* can_names[4] = {"candle0", "candle1", "candle2", "candle3"};
    for (int i = 0; i < 4; ++i) {
        cans[i] = std::make_shared<CANInterface>(can_names[i]);
        rs->BindCAN(cans[i]);
    }

    std::vector<int> motor_indices(12, -1);
    float kp  = 0.0f;   // 零刚度，不产生力
    float kd  = 0.0f;
    float vlim = 44.0f;
    float tlim = 0.0f;   // 零力矩

    // 绑定并使能全部电机（零力矩模式，不会动）
    const int MOTOR_IDS[12] = {1,5,9,13, 2,6,10,14, 3,7,11,15};
    const char CAN_IDS[4] = {'0','1','2','3'};

    for (int j = 0; j < 4; ++j) {
        std::string can_if = std::string("candle") + CAN_IDS[j];
        for (int i = 0; i < 3; ++i) {
            int global_idx = j + i * 4;
            auto minfo = std::make_unique<RobstrideController::MotorInfo>();
            minfo->motor_id = MOTOR_IDS[global_idx];
            minfo->host_id  = 0xFD;
            minfo->max_torque = 17.0f;
            minfo->max_speed  = 44.0f;
            minfo->max_kp = 500.0f;
            minfo->max_kd = 5.0f;

            int idx = rs->BindMotor(can_if.c_str(), std::move(minfo));
            motor_indices[global_idx] = idx;
            rs->EnableMotor(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            rs->EnableAutoReport(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            rs->EnableAutoReport(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            rs->SetMITParams(idx, {kp, kd, vlim, tlim});
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    // 等待数据稳定
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "┌─────────┬──────────────┬────────────┬────────────┬───────────────┐\n";
    std::cout << "│ MPC Idx │ Joint        │ Raw Pos    │ Offset     │ q_mpc         │\n";
    std::cout << "├─────────┼──────────────┼────────────┼────────────┼───────────────┤\n";

    for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
        const JointInfo& ji = JOINTS[mpc_idx];
        int motor_global_idx = MPC_TO_MOTOR_IDX[mpc_idx];
        int motor_idx = motor_indices[motor_global_idx];

        auto ms = rs->GetMotorState(motor_idx);
        double raw_pos = ms.position;
        double offset  = cfg.joint_offsets[motor_global_idx];
        double gear    = ji.is_knee ? cfg.knee_gear_ratio : 1.0;
        double q_mpc   = (raw_pos - offset) / gear;

        std::cout << "│ " << std::setw(7) << mpc_idx
                  << " │ " << std::setw(12) << ji.name
                  << " │ " << std::setw(10) << raw_pos
                  << " │ " << std::setw(10) << offset
                  << " │ " << std::setw(13) << q_mpc
                  << " │\n";
    }

    std::cout << "└─────────┴──────────────┴────────────┴────────────┴───────────────┘\n\n";

    std::cout << "如需标定零偏，将机器人放在标准姿态（四腿垂直），\n"
              << "把 Raw Pos 列的值填入 config/robot_params.yaml 的 joint_offsets。\n\n";

    // 提取便于复制的格式
    std::cout << "# 便于复制的 yaml 格式（当前 Raw Pos 值）：\n";
    std::cout << "joint_offsets:\n";
    std::cout << "  hipa: [";
    for (int leg = 0; leg < 4; ++leg) {
        int mpc_idx = leg * 3;  // HipA
        int motor_global_idx = MPC_TO_MOTOR_IDX[mpc_idx];
        auto ms = rs->GetMotorState(motor_indices[motor_global_idx]);
        std::cout << ms.position;
        if (leg < 3) std::cout << ", ";
    }
    std::cout << "]\n";

    std::cout << "  hipf: [";
    for (int leg = 0; leg < 4; ++leg) {
        int mpc_idx = leg * 3 + 1;  // HipF
        int motor_global_idx = MPC_TO_MOTOR_IDX[mpc_idx];
        auto ms = rs->GetMotorState(motor_indices[motor_global_idx]);
        std::cout << ms.position;
        if (leg < 3) std::cout << ", ";
    }
    std::cout << "]\n";

    std::cout << "  knee: [";
    for (int leg = 0; leg < 4; ++leg) {
        int mpc_idx = leg * 3 + 2;  // Knee
        int motor_global_idx = MPC_TO_MOTOR_IDX[mpc_idx];
        auto ms = rs->GetMotorState(motor_indices[motor_global_idx]);
        std::cout << ms.position;
        if (leg < 3) std::cout << ", ";
    }
    std::cout << "]\n\n";

    // 停止
    for (int i = 0; i < 12; ++i) {
        rs->DisableMotor(motor_indices[i]);
    }

    return 0;
}

// 模式 2: 单关节正弦测试
static int test_single(int mpc_idx, const RobotConfig& cfg) {
    const JointInfo& ji = JOINTS[mpc_idx];

    std::string can_if = std::string("candle") + std::to_string(ji.can_bus);
    std::cout << "\n===== 单关节测试 =====\n"
              << "  MPC index:  " << mpc_idx << "\n"
              << "  关节名称:   " << ji.name << "\n"
              << "  CAN 总线:   " << can_if << "\n"
              << "  Motor ID:   " << ji.motor_id << "\n"
              << "  是否膝关节: " << (ji.is_knee ? "是 (gear=1.667)" : "否") << "\n"
              << "  当前偏移:   " << cfg.joint_offsets[ji.offset_idx] << "\n\n";

    // 初始化
    auto can = std::make_shared<CANInterface>(can_if.c_str());
    auto rs  = std::make_shared<RobstrideController>();
    rs->BindCAN(can);

    auto minfo = std::make_unique<RobstrideController::MotorInfo>();
    minfo->motor_id = ji.motor_id;
    minfo->host_id  = 0xFD;
    minfo->max_torque = 17.0f;
    minfo->max_speed  = 44.0f;
    minfo->max_kp = 500.0f;
    minfo->max_kd = 5.0f;

    int motor_idx = rs->BindMotor(can_if.c_str(), std::move(minfo));
    if (motor_idx < 0) {
        std::cerr << "绑定电机失败！CAN=" << can_if << " ID=" << ji.motor_id << "\n";
        return -1;
    }

    // 低刚度参数
    float kp  = 10.0f;
    float kd  = 2.0f;
    float vlim = 44.0f;
    float tlim = 5.0f;
    rs->SetMITParams(motor_idx, {kp, kd, vlim, tlim});
    rs->EnableMotor(motor_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rs->EnableAutoReport(motor_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    rs->EnableAutoReport(motor_idx);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // 读取初始位置
    auto init_state = rs->GetMotorState(motor_idx);
    float init_pos = init_state.position;
    double offset  = cfg.joint_offsets[ji.offset_idx];
    double gear    = ji.is_knee ? cfg.knee_gear_ratio : 1.0;

    std::cout << "初始 raw position: " << init_pos << " rad\n"
              << "当前 joint_offset: " << offset << "\n"
              << "初始 q_mpc:        " << (init_pos - offset) / gear << " rad\n\n";

    std::cout << "===== 阶段 1: 静止读取 3 秒 =====\n"
              << "观察编码器读数是否稳定。\n\n";

    auto start = std::chrono::steady_clock::now();
    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - start).count();
        if (t > 3.0) break;

        auto ms = rs->GetMotorState(motor_idx);
        std::printf("\r[%.1fs] Raw=%.4f  Vel=%.3f  Tor=%.3f  q_mpc=%.4f",
                    t, ms.position, ms.velocity, ms.torque,
                    (ms.position - offset) / gear);
        std::fflush(stdout);

        // 保持当前位置
        rs->SendMITCommand(motor_idx, init_pos);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    std::cout << "\n\n";

    if (!g_running) goto cleanup;

    {
        std::cout << "===== 阶段 2: 正弦摆动测试 =====\n"
                  << "幅度: ±0.2 rad (MPC空间), 频率: 0.3 Hz, 持续 10 秒\n"
                  << "观察物理关节运动方向是否与预期一致。\n\n"
                  << "  " << ji.name << " 正方向应为: ";

        // 打印预期的物理方向
        if (mpc_idx % 3 == 0) {  // HipA
            if (mpc_idx < 6) std::cout << "左腿外展（向外）\n\n";
            else             std::cout << "右腿外展（向外）\n\n";
        } else if (mpc_idx % 3 == 1) {  // HipF
            std::cout << "前屈（腿向前）\n\n";
        } else {  // Knee
            std::cout << "弯曲（膝盖弯曲）\n\n";
        }

        auto sine_start = std::chrono::steady_clock::now();
        double amplitude_mpc = 0.2;  // ±0.2 rad in MPC space
        double freq = 0.3;           // 0.3 Hz

        while (g_running) {
            auto now = std::chrono::steady_clock::now();
            double t = std::chrono::duration<double>(now - sine_start).count();
            if (t > 10.0) break;

            double q_mpc_cmd = amplitude_mpc * std::sin(2.0 * M_PI * freq * t);
            double raw_cmd   = q_mpc_cmd * gear + offset;

            rs->SendMITCommand(motor_idx, (float)raw_cmd);

            auto ms = rs->GetMotorState(motor_idx);
            double q_mpc_now = (ms.position - offset) / gear;

            std::printf("\r[%.1fs] cmd_mpc=%.3f  act_mpc=%.3f  raw=%.4f  tor=%.3f",
                        t, q_mpc_cmd, q_mpc_now, ms.position, ms.torque);
            std::fflush(stdout);

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        std::cout << "\n\n";
    }

cleanup:
    std::cout << "===== 停止电机 =====\n";
    // 缓慢回到初始位置
    auto ms_now = rs->GetMotorState(motor_idx);
    float current_pos = ms_now.position;
    for (int step = 1; step <= 20; ++step) {
        float p = current_pos + (init_pos - current_pos) * (float)step / 20.0f;
        rs->SendMITCommand(motor_idx, p);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    rs->DisableMotor(motor_idx);
    std::cout << "电机已停止。\n\n";

    return 0;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    if (argc < 2) {
        print_usage();
        return 1;
    }

    std::string config_path = (argc > 2) ? argv[2] : "config/robot_params.yaml";
    RobotConfig cfg = RobotConfig::from_yaml(config_path);

    std::string arg1 = argv[1];
    if (arg1 == "all") {
        return read_all_positions(cfg);
    }

    int mpc_idx = std::stoi(arg1);
    if (mpc_idx < 0 || mpc_idx > 11) {
        std::cerr << "mpc_index 必须在 0-11 之间\n";
        print_usage();
        return 1;
    }

    return test_single(mpc_idx, cfg);
}
