// =============================================================================
// MPC Robot Control - Main Entry Point
// =============================================================================
// Architecture: dual-rate loop
//   500Hz PD thread  - state estimation, gait, WBC, motor commands (real-time)
//   30Hz  MPC thread - NMPC footstep planning (non-realtime)
//
// Hardware:
//   Motors: Robstride via 4x CAN interfaces (candle0..3), MIT mode
//   IMU:    WIT-Motion via serial (/dev/ttyCH341USB0)
//   Input:  Linux joystick (/dev/input/js0) or keyboard
//
// Reuses from pure_cpp/:
//   RobstrideController, CANInterface, IMUComponent, Gamepad
// =============================================================================

#include <iostream>
#include <thread>
#include <mutex>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <unistd.h>
#include <sys/select.h>
#include <sys/mman.h>
#include <pthread.h>
#include <sched.h>

// Pure_cpp hardware layer
#include "robstride.hpp"
#include "observations.hpp"  // IMUComponent, Gamepad, JointComponent

// MPC controllers
#include "common/robot_config.hpp"
#include "common/robot_state.hpp"
#include "common/math_utils.hpp"
#include "kinematics/quadruped_kin.hpp"
#include "estimator/state_estimator.hpp"
#include "gait/gait_generator.hpp"
#include "mpc/mpc_controller.hpp"
#include "mpc/nmpc_footstep.hpp"
#include "wbc/wbc_controller.hpp"

// ============================================================
// Globals
// ============================================================
static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// Motor index reordering:
//   motor_indices[] order (matches hardware.motor_ids in yaml):
//     [LF_HipA, LR_HipA, RF_HipA, RR_HipA,
//      LF_HipF, LR_HipF, RF_HipF, RR_HipF,
//      LF_Knee, LR_Knee, RF_Knee, RR_Knee]
//   MPC joint order:
//     [LF_HipA, LF_HipF, LF_Knee,
//      LR_HipA, LR_HipF, LR_Knee,
//      RF_HipA, RF_HipF, RF_Knee,
//      RR_HipA, RR_HipF, RR_Knee]
//   MPC_TO_MOTOR_IDX[mpc_idx] -> motor_indices[] subscript
static const int MPC_TO_MOTOR_IDX[12] = {0,4,8, 1,5,9, 2,6,10, 3,7,11};

// Per-joint position limits in RAW motor space (with offset, before gear-ratio removal).
// Copied from pure_cpp/Main/src/main.cpp (xml_min / xml_max).
// Index order: [LF_HipA, LR_HipA, RF_HipA, RR_HipA,
//               LF_HipF, LR_HipF, RF_HipF, RR_HipF,
//               LF_Knee, LR_Knee, RF_Knee, RR_Knee]
// Per-joint limits expressed as MOTOR SHAFT DISPLACEMENT from stand pose:
//   shaft_disp = raw_encoder - joint_offset
// At stand pose shaft_disp = 0. Knee values include the gear ratio (joint * 1.667).
// Index order: [LF_HipA, LR_HipA, RF_HipA, RR_HipA,
//               LF_HipF, LR_HipF, RF_HipF, RR_HipF,
//               LF_Knee, LR_Knee, RF_Knee, RR_Knee]
static const float JOINT_SHAFT_MIN[12] = {
    -0.7853982f, -0.7853982f, -0.7853982f, -0.7853982f,  // HipA  (gear=1, same as joint)
    -1.2217658f, -1.2217305f, -0.8726999f, -0.8726999f,  // HipF  (gear=1)
    -1.2217299f * 1.667f, -1.2217299f * 1.667f, -0.6f, -0.6f  // Knee shaft
};
static const float JOINT_SHAFT_MAX[12] = {
     0.7853982f,  0.7853982f,  0.7853982f,  0.7853982f,  // HipA
     0.8726683f,  0.8726683f,  1.2217342f,  1.2217305f,  // HipF
     0.6f, 0.6f, 1.2217287f * 1.667f, 1.2217287f * 1.667f  // Knee shaft
};
// Emergency stop margin applied on top of shaft limits [rad]
static constexpr float JOINT_LIMIT_MARGIN = 0.1f;

// ============================================================
// Motor startup: initialize all 12 motors, interpolate to stand pose
// ============================================================
static void startup_motors(std::shared_ptr<RobstrideController> rs,
                             std::vector<int>& motor_indices,
                             const RobotConfig& cfg)
{
    float kp  = (float)cfg.mit_kp;
    float kd  = (float)cfg.mit_kd;
    float vlim = (float)cfg.mit_vel_limit;
    float tlim = (float)cfg.mit_torque_limit;

    // Bind and enable motors in hardware.motor_ids order
    // j = CAN bus index (0=LF,1=LR,2=RF,3=RR), i = joint index (0=HipA,1=HipF,2=Knee)
    if ((int)cfg.can_interfaces.size() < 4) {
        std::cerr << "[Startup] ERROR: need 4 CAN interfaces, got "
                  << cfg.can_interfaces.size() << "\n";
        return;
    }
    for (int j = 0; j < 4; ++j) {
        const std::string& can_if = cfg.can_interfaces[j];
        for (int i = 0; i < 3; ++i) {
            int global_idx = j + i * 4;
            auto minfo = std::make_unique<RobstrideController::MotorInfo>();
            minfo->motor_id = cfg.motor_ids[global_idx];
            minfo->host_id  = 0xFD;

            int idx = rs->BindMotor(can_if.c_str(), std::move(minfo));
            if (idx < 0) {
                std::cerr << "[Startup] ERROR: BindMotor failed for motor_id="
                          << cfg.motor_ids[global_idx]
                          << " on " << can_if << " (CAN interface not found?)\n";
                return;
            }
            motor_indices[global_idx] = idx;
            rs->EnableMotor(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            rs->EnableAutoReport(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            rs->EnableAutoReport(idx);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            rs->SetMITParams(idx, {kp, kd, vlim, tlim});
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    // Wait for first encoder feedback from all motors before reading positions
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Read current encoder positions as interpolation start points.
    // This avoids the jolt that would occur if we always interpolated from 0.
    float start_pos[12];
    for (int global_idx = 0; global_idx < 12; ++global_idx) {
        start_pos[global_idx] = rs->GetMotorState(motor_indices[global_idx]).position;
    }
    std::cout << "[Startup] Interpolating from current encoder positions to stand pose...\n";

    // Interpolate all joints simultaneously: 100 steps × 20ms = 2 seconds
    const int STEPS = 100;
    for (int step = 1; step <= STEPS && g_running; ++step) {
        float alpha = (float)step / STEPS;
        for (int global_idx = 0; global_idx < 12; ++global_idx) {
            float target = start_pos[global_idx]
                         + alpha * ((float)cfg.joint_offsets[global_idx] - start_pos[global_idx]);
            rs->SendMITCommand(motor_indices[global_idx], target);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (g_running)
        std::cout << "[Startup] Motors at stand pose.\n";
}

// ============================================================
// IMU sensor read
// ============================================================
// WIT IMU frame: X=right, Y=forward, Z=up.
// Quaternion order: [q0=w, q1=x, q2=y, q3=z].
// StateEstimator applies R_imu2body = [[0,1,0],[-1,0,0],[0,0,1]] internally.
static void read_sensors(const std::shared_ptr<IMUComponent>& imu_comp,
                          IMUSensorData& imu_data)
{
    const float* q = imu_comp->get_quaternion();
    const float* g = imu_comp->get_gyro();
    const float* a = imu_comp->get_acc();
    imu_data.quat      << q[0], q[1], q[2], q[3];
    imu_data.gyro_imu  << g[0], g[1], g[2];
    imu_data.accel_imu << a[0], a[1], a[2];
}

// ============================================================
// Joint read + per-joint limit safety check (single motor state read per joint)
// ============================================================
// Each joint's motor_state is fetched exactly once to avoid:
//   (a) redundant mutex acquisitions (was: safety check + read_joints = 2x per joint)
//   (b) reading different values for the same joint across the two passes
//
// Returns false and prints an error if any joint exceeds its raw position limit.
// Caller must trigger emergency stop when false is returned.
//
// motor_indices order: [LF_HipA, LR_HipA, RF_HipA, RR_HipA,
//                       LF_HipF, LR_HipF, RF_HipF, RR_HipF,
//                       LF_Knee, LR_Knee, RF_Knee, RR_Knee]
// MPC order:           [LF_HipA, LF_HipF, LF_Knee,
//                       LR_HipA, LR_HipF, LR_Knee, ...]
static bool read_joints_safe(const std::shared_ptr<RobstrideController>& rs,
                              const std::vector<int>& motor_indices,
                              const RobotConfig& cfg,
                              Eigen::Matrix<double,12,1>& q_mpc,
                              Eigen::Matrix<double,12,1>& dq_mpc,
                              Eigen::Matrix<double,12,1>& tau_est)
{
    bool ok = true;
    for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
        int gi        = MPC_TO_MOTOR_IDX[mpc_idx];
        int motor_idx = motor_indices[gi];

        // Single read — holds motor_data_mutex for one call only
        auto ms = rs->GetMotorState(motor_idx);

        // --- Limit check in shaft-displacement space (raw - offset) ---
        double offset      = cfg.joint_offsets[gi];
        float shaft_disp   = ms.position - (float)offset;
        if (shaft_disp < JOINT_SHAFT_MIN[gi] - JOINT_LIMIT_MARGIN ||
            shaft_disp > JOINT_SHAFT_MAX[gi] + JOINT_LIMIT_MARGIN) {
            std::cerr << "[SAFETY] Joint " << mpc_idx
                      << " (motor_id=" << cfg.motor_ids[gi]
                      << ") shaft_disp=" << shaft_disp
                      << " outside safe range ["
                      << JOINT_SHAFT_MIN[gi] << ", " << JOINT_SHAFT_MAX[gi]
                      << "] — EMERGENCY STOP\n";
            ok = false;
            // Continue loop to report all violated joints before stopping
        }

        // --- Convert to MPC joint space ---
        bool is_knee   = (mpc_idx % 3 == 2);
        if (is_knee) {
            q_mpc[mpc_idx]  = (ms.position - offset) / cfg.knee_gear_ratio;
            dq_mpc[mpc_idx] = ms.velocity / cfg.knee_gear_ratio;
        } else {
            q_mpc[mpc_idx]  = ms.position - offset;
            dq_mpc[mpc_idx] = ms.velocity;
        }
        tau_est[mpc_idx] = std::abs(ms.torque);
    }
    return ok;
}

// ============================================================
// Send full MIT commands: position + velocity + PD gains + feedforward torque
// ============================================================
// The Robstride MIT mode executes at ~10kHz on the motor driver:
//   τ_motor = kp*(pos_des - pos) + kd*(vel_des - vel) + tau_ff
//
// For stance legs: pos=stand angle, kp/kd=stance gains, tau_ff=WBC torque (J^T * f_mpc)
// For swing legs:  pos=swing target, kp/kd=swing gains, tau_ff=0
//
// Knee joints have gear_ratio=1.667, requiring conversion:
//   pos_motor = pos_joint * gear + offset
//   kp_motor  = kp_joint / gear^2    (so joint sees correct stiffness)
//   kd_motor  = kd_joint / gear^2
//   tau_motor = tau_joint / gear      (power conservation: τ₁ω₁ = τ₂ω₂)
static void send_motor_commands(const std::shared_ptr<RobstrideController>& rs,
                                 const std::vector<int>& motor_indices,
                                 const RobotConfig& cfg,
                                 const Eigen::Matrix<double,12,1>& q_des,
                                 const Eigen::Matrix<double,12,1>& tau_ff,
                                 const Eigen::Matrix<double,12,1>& kp_vec,
                                 const Eigen::Matrix<double,12,1>& kd_vec)
{
    for (int mpc_idx = 0; mpc_idx < 12; ++mpc_idx) {
        int motor_global_idx = MPC_TO_MOTOR_IDX[mpc_idx];
        int motor_idx = motor_indices[motor_global_idx];
        double offset = cfg.joint_offsets[motor_global_idx];

        bool is_knee = (mpc_idx % 3 == 2);
        double gear  = is_knee ? cfg.knee_gear_ratio : 1.0;
        double gear2 = gear * gear;

        // Convert joint space → motor shaft space
        // Clamp shaft displacement (= joint * gear) before adding offset
        double shaft_cmd = math_utils::clamp(q_des[mpc_idx] * gear,
                                              (double)JOINT_SHAFT_MIN[motor_global_idx],
                                              (double)JOINT_SHAFT_MAX[motor_global_idx]);
        double pos_raw = shaft_cmd + offset;
        double kp_raw  = kp_vec[mpc_idx] / gear2;
        double kd_raw  = kd_vec[mpc_idx] / gear2;
        double tau_raw = tau_ff[mpc_idx] / gear;

        // Clamp torque feedforward to hardware limit
        tau_raw = math_utils::clamp(tau_raw,
                                     -cfg.mit_torque_limit,
                                      cfg.mit_torque_limit);

        // Clamp kp/kd to Robstride register limits (kp: 0~500, kd: 0~5)
        kp_raw = math_utils::clamp(kp_raw, 0.0, 500.0);
        kd_raw = math_utils::clamp(kd_raw, 0.0, 5.0);

        rs->SendMITCommand(motor_idx, (float)pos_raw, 0.0f,
                           (float)kp_raw, (float)kd_raw, (float)tau_raw);
    }
}

// ============================================================
// MPC Thread (30Hz)
// ============================================================
struct SharedMPCData {
    MPCOutput output;
    Eigen::Matrix<double, 4, 3> foot_target;   // Raibert foot placement targets
    // Written by PD thread, read by MPC thread
    Eigen::Vector3d v_cmd_body{0., 0., 0.};
    double yaw_rate_cmd{0.0};
    double gait_phase{0.0};
    bool gait_active{false};   // false = STAND (all-contact MPC), true = WALK (trot contact)
    std::chrono::steady_clock::time_point last_update{std::chrono::steady_clock::now()};
    std::mutex mtx;
    bool initialized = false;
};

// MPC thread: ConvexMPC (OSQP) for GRF + Raibert heuristic for foot placement.
// NMPC is intentionally excluded — it runs at ~120ms on Jetson which breaks 30Hz timing.
static void mpc_thread_fn(RobotConfig cfg,
                           SharedMPCData& shared,
                           std::atomic<bool>& running,
                           const RobotState& state_ref,
                           std::mutex& state_mtx)
{
    MPCController mpc(cfg);
    GaitGenerator gait_mpc(cfg);  // separate instance; phase synced from PD thread

    auto period = std::chrono::microseconds(static_cast<int>(1e6 / cfg.mpc_freq));
    auto next_wake = std::chrono::steady_clock::now();

    while (running) {
        next_wake += period;

        // Read robot state
        RobotState state;
        {
            std::lock_guard<std::mutex> lk(state_mtx);
            state = state_ref;
        }

        // Read velocity command, gait phase, and active flag written by PD thread
        Eigen::Vector3d v_cmd_body;
        double yaw_rate_cmd;
        bool is_gait_active;
        {
            std::lock_guard<std::mutex> lk(shared.mtx);
            v_cmd_body     = shared.v_cmd_body;
            yaw_rate_cmd   = shared.yaw_rate_cmd;
            is_gait_active = shared.gait_active;
            gait_mpc.set_phase(shared.gait_phase);
        }

        // Predict contact sequence and build MPC reference
        Eigen::MatrixXd X_ref = gait_mpc.get_mpc_reference(
            state, cfg.target_z, v_cmd_body, yaw_rate_cmd);

        // Contact: in STAND mode force all-four-contact so MPC distributes weight evenly.
        // In WALK mode use the trot gait prediction.
        std::array<bool, 4> contact_now;
        if (!is_gait_active) {
            contact_now = {true, true, true, true};
        } else {
            Eigen::MatrixXd contact_seq = gait_mpc.predict_contact_sequence(cfg.mpc_horizon);
            for (int i = 0; i < 4; ++i) contact_now[i] = (contact_seq(i, 0) > 0.5);
        }

        // Solve ConvexMPC for GRF
        Eigen::Matrix<double, 12, 1> f_grf =
            mpc.solve(state, state.foot_pos_world, contact_now, X_ref);

        // Raibert heuristic: place swing foot at hip + v_cmd * T_stance/2
        double T_stance = cfg.duty_factor * cfg.gait_period;
        double yaw = state.rpy[2];
        Eigen::Vector3d v_world{
            v_cmd_body[0]*std::cos(yaw) - v_cmd_body[1]*std::sin(yaw),
            v_cmd_body[0]*std::sin(yaw) + v_cmd_body[1]*std::cos(yaw),
            0.0
        };

        Eigen::Matrix<double, 4, 3> foot_targets = state.foot_pos_world;  // stance: hold current
        for (int i = 0; i < 4; ++i) {
            if (!contact_now[i]) {
                // Hip position in world frame
                foot_targets(i, 0) = state.pos[0]
                    + cfg.hip_offsets(i, 0) * std::cos(yaw)
                    - cfg.hip_offsets(i, 1) * std::sin(yaw)
                    + v_world[0] * T_stance * 0.5;
                foot_targets(i, 1) = state.pos[1]
                    + cfg.hip_offsets(i, 0) * std::sin(yaw)
                    + cfg.hip_offsets(i, 1) * std::cos(yaw)
                    + v_world[1] * T_stance * 0.5;
                foot_targets(i, 2) = 0.0;
            }
        }

        // Publish to PD thread
        {
            std::lock_guard<std::mutex> lk(shared.mtx);
            for (int i = 0; i < 4; ++i)
                shared.output.f_stance.row(i) = f_grf.segment<3>(i * 3).transpose();
            shared.foot_target          = foot_targets;
            shared.output.solve_time_ms = mpc.last_solve_ms();
            shared.output.valid         = true;
            shared.initialized          = true;
            shared.last_update          = std::chrono::steady_clock::now();
        }

        if (mpc.last_solve_ms() > 10.0)
            std::cerr << "[MPC] Slow solve: " << mpc.last_solve_ms() << " ms\n";

        std::this_thread::sleep_until(next_wake);
    }
}

// ============================================================
// Main
// ============================================================
int main(int argc, char* argv[]) {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // Lock all memory pages to prevent page faults in realtime thread
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        std::cerr << "[WARN] mlockall failed (run as root for real-time performance)\n";
    }
    // Pre-touch stack: use 512KB (safe within default 8MB stack limit).
    // The original 8MB allocation overflows the default Jetson stack.
    // Increase system stack first if a larger pre-fault is needed:
    //   sudo bash -c 'echo "* soft stack unlimited" >> /etc/security/limits.conf'
    volatile char stack_prefault[512 * 1024];
    std::memset((void*)stack_prefault, 0, sizeof(stack_prefault));

    // Load configuration (non-const: nominal_foot_offsets will be calibrated at runtime)
    std::string config_path = (argc > 1) ? argv[1] : "config/robot_params.yaml";
    RobotConfig cfg = RobotConfig::from_yaml(config_path);
    std::cout << "[Init] Config loaded from " << config_path << "\n";

    // Initialize kinematics
    QuadrupedKinematics kin(cfg);
    // NOTE: cfg.nominal_foot_offsets will be updated at t=calib_duration from actual kinematics.

    // ===== Hardware initialization =====
    auto rs = std::make_shared<RobstrideController>();
    auto can0 = std::make_shared<CANInterface>("candle0");
    auto can1 = std::make_shared<CANInterface>("candle1");
    auto can2 = std::make_shared<CANInterface>("candle2");
    auto can3 = std::make_shared<CANInterface>("candle3");
    rs->BindCAN(can0); rs->BindCAN(can1); rs->BindCAN(can2); rs->BindCAN(can3);

    std::vector<int> motor_indices(12);
    startup_motors(rs, motor_indices, cfg);

    if (!g_running) {
        std::cout << "[Shutdown] Interrupted during startup.\n";
        for (int i = 0; i < 12; ++i) {
            rs->SendMITCommand(motor_indices[i], 0.0f);
            rs->DisableMotor(motor_indices[i]);
        }
        return 0;
    }
    std::cout << "Press ENTER to start MPC control loop (Ctrl+C to abort)...\n";
    // Poll stdin every 100ms so Ctrl+C is detected without waiting for ENTER.
    // Plain cin.get() is restarted by the kernel after SIGINT (SA_RESTART),
    // so it never returns on Ctrl+C alone.
    {
        bool entered = false;
        while (g_running && !entered) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(STDIN_FILENO, &fds);
            struct timeval tv{0, 100000};  // 100ms
            if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) {
                std::cin.get();
                entered = true;
            }
        }
    }
    if (!g_running) {
        std::cout << "[Shutdown] Interrupted at ENTER prompt.\n";
        for (int i = 0; i < 12; ++i) {
            rs->SendMITCommand(motor_indices[i], 0.0f);
            rs->DisableMotor(motor_indices[i]);
        }
        return 0;
    }

    auto imu_comp = std::make_shared<IMUComponent>(cfg.imu_device.c_str());
    auto gamepad  = std::make_shared<Gamepad>(cfg.gamepad_dev.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // let IMU settle

    // ===== Control modules =====
    StateEstimator estimator(cfg, kin);
    GaitGenerator  gait(cfg);
    WBCController  wbc(cfg, kin);

    // ===== Shared data =====
    SharedMPCData  shared_mpc;
    RobotState     shared_state;
    std::mutex     state_mtx;

    // ===== MPC thread =====
    std::thread mpc_th(mpc_thread_fn, cfg,   // pass by value (copy)
                       std::ref(shared_mpc), std::ref(g_running),
                       std::cref(shared_state), std::ref(state_mtx));

    // ===== Main loop setup =====
    // Startup calibration state
    bool calib_done = false;
    double startup_time = 0.0;

    // Cached MPC output (updated every ~33ms)
    MPCOutput mpc_cache;
    Eigen::Matrix<double,4,3> foot_target_cache = Eigen::Matrix<double,4,3>::Zero();

    // Swing tracking
    Eigen::Matrix<double,12,1> q_des_swing = Eigen::Matrix<double,12,1>::Zero();
    std::array<bool,4> prev_contact{true,true,true,true};

    // Velocity command (read from gamepad)
    Eigen::Vector3d v_cmd_body{0.,0.,0.};
    double yaw_rate_cmd = 0.0;

    // Gait activation: stays in MPC standing mode until gamepad commands velocity
    bool gait_active = false;
    constexpr double GAIT_ACTIVATE_THRESHOLD = 0.03;  // m/s or rad/s

    // ===== Elevate PD thread priority =====
    {
        struct sched_param param;
        param.sched_priority = 90;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
            std::cerr << "[WARN] Failed to set SCHED_FIFO (run as root)\n";
        }
    }

    std::cout << "[Control] Starting 500Hz control loop\n";

    auto loop_start = std::chrono::steady_clock::now();
    auto next_500hz = loop_start;
    auto last_print = loop_start;

    while (g_running) {
        next_500hz += std::chrono::microseconds(2000);  // 500Hz = 2ms

        auto now = std::chrono::steady_clock::now();
        startup_time = std::chrono::duration<double>(now - loop_start).count();
        double dt = 0.002;  // nominal 2ms

        // ===== 1. Read sensors =====
        IMUSensorData imu_data;
        read_sensors(imu_comp, imu_data);

        // Read all joint states once; also performs per-joint limit check.
        // Motor state mutex is acquired exactly once per joint (12 total).
        Eigen::Matrix<double,12,1> q_mpc, dq_mpc, tau_est;
        if (!read_joints_safe(rs, motor_indices, cfg, q_mpc, dq_mpc, tau_est)) {
            g_running = false; break;
        }

        // ===== 2. State estimation =====
        estimator.update(imu_data, q_mpc, dq_mpc, tau_est, dt, startup_time);
        const RobotState& state = estimator.state();

        // ===== 2b. Attitude emergency stop =====
        // If the robot has fallen over, continuing to run MPC causes violent thrashing.
        // Stop immediately when body tilt exceeds safe threshold.
        if (calib_done) {
            constexpr double TILT_LIMIT = 0.8;  // ~46 deg — well beyond any normal operation
            if (std::abs(state.rpy[0]) > TILT_LIMIT ||
                std::abs(state.rpy[1]) > TILT_LIMIT) {
                std::cerr << "[SAFETY] Excessive tilt: roll=" << state.rpy[0]
                          << " pitch=" << state.rpy[1] << " — EMERGENCY STOP\n";
                g_running = false;
                break;
            }
        }

        // ===== 3. Update shared state for MPC thread =====
        {
            std::lock_guard<std::mutex> lk(state_mtx);
            shared_state = state;
        }

        // ===== 4. Read gamepad =====
        if (gamepad->IsConnected()) {
            v_cmd_body[0] = -gamepad->GetAxis(1) * 0.5;  // forward
            v_cmd_body[1] =  0.0;
            yaw_rate_cmd  = -gamepad->GetAxis(3) * 0.5;  // yaw
        }
        // Share velocity command with MPC thread (written every 2ms, read every 33ms)
        {
            std::lock_guard<std::mutex> lk(shared_mpc.mtx);
            shared_mpc.v_cmd_body   = v_cmd_body;
            shared_mpc.yaw_rate_cmd = yaw_rate_cmd;
        }

        // ===== 5. Calibration phase (first calib_duration seconds) =====
        if (startup_time < cfg.calib_duration) {
            // Hold at stand pose with high stiffness
            // Phase B: PD stand — motor holds at zero offset position, no feedforward torque
            Eigen::Matrix<double,12,1> q_des_stand = Eigen::Matrix<double,12,1>::Zero();
            Eigen::Matrix<double,12,1> tau_ff_stand = Eigen::Matrix<double,12,1>::Zero();
            Eigen::Matrix<double,12,1> kp_vec = Eigen::Matrix<double,12,1>::Constant(cfg.kp_stand);
            Eigen::Matrix<double,12,1> kd_vec = Eigen::Matrix<double,12,1>::Constant(cfg.kd_stand);
            send_motor_commands(rs, motor_indices, cfg, q_des_stand, tau_ff_stand, kp_vec, kd_vec);

            if (startup_time > cfg.calib_duration - 0.05 && !calib_done) {
                // Calibrate nominal foot offsets from current kinematics
                cfg.nominal_foot_offsets = state.foot_pos_body;
                q_des_swing = q_mpc;
                calib_done = true;
                std::cout << "[Calib] Nominal foot offsets calibrated.\n";
            }

            std::this_thread::sleep_until(next_500hz);
            continue;
        }

        // ===== 6. Read MPC cache =====
        {
            std::lock_guard<std::mutex> lk(shared_mpc.mtx);
            if (shared_mpc.initialized) {
                mpc_cache         = shared_mpc.output;
                foot_target_cache = shared_mpc.foot_target;

                // MPC staleness check: if last solve was > 500ms ago, the MPC thread
                // has likely hung. Zero out GRF to fall back to pure PD holding.
                auto age_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - shared_mpc.last_update).count();
                if (age_ms > 500 && calib_done) {
                    std::cerr << "[SAFETY] MPC output stale (" << age_ms
                              << "ms) — zeroing GRF\n";
                    mpc_cache.f_stance.setZero();
                }
            }
        }

        // ===== 7. Gait activation check =====
        // Gait stays inactive until gamepad commands non-zero velocity.
        // This gives the operator time to verify MPC standing before walking.
        if (!gait_active) {
            double v_norm = v_cmd_body.head<2>().norm();
            if (v_norm > GAIT_ACTIVATE_THRESHOLD ||
                std::abs(yaw_rate_cmd) > GAIT_ACTIVATE_THRESHOLD) {
                gait_active = true;
                std::cout << "[Control] Gait ACTIVATED (gamepad input detected)\n";
                std::lock_guard<std::mutex> lk(shared_mpc.mtx);
                shared_mpc.gait_active = true;
            }
        }

        // ===== 8. Gait + WBC + Motor targets (500Hz) =====
        Eigen::Matrix<double,12,1> tau_ff = Eigen::Matrix<double,12,1>::Zero();
        Eigen::Matrix<double,12,1> q_des  = Eigen::Matrix<double,12,1>::Zero();
        Eigen::Matrix<double,12,1> kp_vec = Eigen::Matrix<double,12,1>::Zero();
        Eigen::Matrix<double,12,1> kd_vec = Eigen::Matrix<double,12,1>::Zero();

        if (!gait_active) {
            // ---- MPC Standing mode ----
            // All 4 legs in contact, MPC provides GRF, no gait.
            // Operator can observe MPC GRF and verify correctness.
            std::array<bool,4> all_contact{true, true, true, true};

            WBCController::Input wbc_in;
            wbc_in.f_mpc     = mpc_cache.f_stance;
            wbc_in.foot_vel_w = state.foot_vel_world;
            wbc_in.dq        = dq_mpc;
            wbc_in.contact   = all_contact;
            wbc_in.state     = &state;

            Eigen::Matrix<double,12,1> tau_wbc = wbc.compute(wbc_in);

            for (int i = 0; i < 4; ++i) {
                int idx = i * 3;
                q_des.segment<3>(idx)  = cfg.stand_joint_angles.segment<3>(idx);
                kp_vec.segment<3>(idx).setConstant(cfg.kp_stance);
                kd_vec.segment<3>(idx).setConstant(cfg.kd_stance);
                tau_ff.segment<3>(idx) = tau_wbc.segment<3>(idx);
            }

            // Keep gait phase frozen at 0, share with MPC thread
            {
                std::lock_guard<std::mutex> lk(shared_mpc.mtx);
                shared_mpc.gait_phase = 0.0;
            }
        } else {
            // ---- Walking mode ----
            double yaw = state.rpy[2];
            Eigen::Vector3d v_cmd_world{
                v_cmd_body[0]*std::cos(yaw) - v_cmd_body[1]*std::sin(yaw),
                v_cmd_body[0]*std::sin(yaw) + v_cmd_body[1]*std::cos(yaw),
                0.0
            };

            GaitOutput gait_out = gait.update(dt, state, v_cmd_world, foot_target_cache);

            // Share gait phase with MPC thread
            {
                std::lock_guard<std::mutex> lk(shared_mpc.mtx);
                shared_mpc.gait_phase = gait.phase();
            }

            // WBC
            WBCController::Input wbc_in;
            wbc_in.f_mpc     = mpc_cache.f_stance;
            wbc_in.foot_vel_w = state.foot_vel_world;
            wbc_in.dq        = dq_mpc;
            for (int i = 0; i < 4; ++i) wbc_in.contact[i] = gait_out.contact[i];
            wbc_in.state = &state;

            Eigen::Matrix<double,12,1> tau_wbc = wbc.compute(wbc_in);

            // Motor targets: stance legs get WBC tau_ff, swing legs get IK tracking
            for (int i = 0; i < 4; ++i) {
                int idx = i * 3;
                if (gait_out.contact[i]) {
                    q_des.segment<3>(idx)  = cfg.stand_joint_angles.segment<3>(idx);
                    kp_vec.segment<3>(idx).setConstant(cfg.kp_stance);
                    kd_vec.segment<3>(idx).setConstant(cfg.kd_stance);
                    tau_ff.segment<3>(idx) = tau_wbc.segment<3>(idx);
                } else {
                    // Swing: IK-based trajectory tracking
                    if (prev_contact[i]) {
                        q_des_swing.segment<3>(idx) = q_mpc.segment<3>(idx);
                    }

                    Eigen::Vector3d p_err = gait_out.foot_target_pos.row(i).transpose()
                                           - state.foot_pos_world.row(i).transpose();
                    Eigen::Vector3d v_cart_cmd = gait_out.foot_target_vel.row(i).transpose()
                                                + 20.0 * p_err;

                    Eigen::Matrix3d J_body  = kin.jacobian(i, q_mpc.segment<3>(idx));
                    Eigen::Matrix3d J_world = state.rot_mat * J_body;
                    Eigen::Matrix3d JJT     = J_world * J_world.transpose()
                                             + 1e-3 * Eigen::Matrix3d::Identity();
                    Eigen::Matrix3d J_inv   = J_world.transpose() * JJT.inverse();
                    Eigen::Vector3d dq_des  = J_inv * v_cart_cmd;

                    q_des_swing.segment<3>(idx) += dq_des * dt;

                    // Anti-windup: clamp q_des_swing to per-joint MPC-space limits
                    // so the integrator doesn't drift beyond the physical range.
                    // MPC-space limit = shaft_disp_limit / gear
                    // (JOINT_SHAFT_MIN/MAX are already relative to stand pose = offset)
                    for (int j = 0; j < 3; ++j) {
                        int gi_j  = MPC_TO_MOTOR_IDX[idx + j];
                        bool knee = (j == 2);
                        double gr = knee ? cfg.knee_gear_ratio : 1.0;
                        double lo = (double)JOINT_SHAFT_MIN[gi_j] / gr;
                        double hi = (double)JOINT_SHAFT_MAX[gi_j] / gr;
                        q_des_swing[idx + j] = math_utils::clamp(q_des_swing[idx + j], lo, hi);
                    }

                    q_des.segment<3>(idx)  = q_des_swing.segment<3>(idx);
                    kp_vec.segment<3>(idx).setConstant(cfg.kp_swing);
                    kd_vec.segment<3>(idx).setConstant(cfg.kd_swing);
                }
            }

            for (int i = 0; i < 4; ++i) prev_contact[i] = gait_out.contact[i];
        }

        // Safety: NaN/Inf check — zero bad outputs rather than sending them to motors
        if (tau_ff.hasNaN() || !tau_ff.allFinite()) tau_ff.setZero();
        if (q_des.hasNaN()  || !q_des.allFinite())  q_des = Eigen::Matrix<double,12,1>::Zero();

        // ===== 9. Send motor commands =====
        send_motor_commands(rs, motor_indices, cfg, q_des, tau_ff, kp_vec, kd_vec);

        // ===== 10. Debug print (every 0.5s) =====
        if (std::chrono::duration<double>(now - last_print).count() >= 0.5) {
            std::printf("[%6.1fs] %s | vx=%.2f yaw=%.2f | mpc_ms=%.1f | z=%.3f",
                        startup_time,
                        gait_active ? "WALK" : "STAND",
                        v_cmd_body[0], yaw_rate_cmd,
                        mpc_cache.solve_time_ms, state.pos[2]);
            if (!gait_active) {
                // Print MPC GRF for operator verification
                std::printf(" | fz=[%.1f,%.1f,%.1f,%.1f]N",
                            mpc_cache.f_stance(0,2), mpc_cache.f_stance(1,2),
                            mpc_cache.f_stance(2,2), mpc_cache.f_stance(3,2));
            }
            std::printf("\n");
            last_print = now;
        }

        std::this_thread::sleep_until(next_500hz);
    }

    // ===== Shutdown =====
    std::cout << "[Shutdown] Stopping...\n";
    g_running = false;
    if (mpc_th.joinable()) mpc_th.join();

    // Disable all motors safely
    for (int i = 0; i < 12; ++i) {
        rs->SendMITCommand(motor_indices[i], 0.0f);
        rs->DisableMotor(motor_indices[i]);
    }

    std::cout << "[Shutdown] Done.\n";
    return 0;
}
