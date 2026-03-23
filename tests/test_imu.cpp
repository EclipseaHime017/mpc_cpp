// =============================================================================
// IMU Verification Tool
// Usage: sudo ./build/test_imu [config_path]
//
// Reads WIT-Motion IMU data and prints:
//   - Raw quaternion, gyro, accel (WIT frame)
//   - Converted rpy, omega, gravity (body frame, after R_imu2body)
// No motors involved. Safe to run anytime.
// =============================================================================

#include <iostream>
#include <iomanip>
#include <cmath>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <memory>

#include "observations.hpp"
#include "common/robot_config.hpp"
#include "common/math_utils.hpp"

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

int main(int argc, char* argv[]) {
    std::signal(SIGINT, signal_handler);

    std::string config_path = (argc > 1) ? argv[1] : "config/robot_params.yaml";
    RobotConfig cfg = RobotConfig::from_yaml(config_path);

    std::cout << "=== IMU Verification Tool ===\n";
    std::cout << "Device: " << cfg.imu_device << "\n";
    std::cout << "R_imu2body: X_body = IMU_Y, Y_body = -IMU_X, Z_body = IMU_Z\n\n";
    std::cout << "Press Ctrl+C to stop.\n\n";

    auto imu = std::make_shared<IMUComponent>(cfg.imu_device.c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // let IMU settle

    // R_imu2body = [[0,1,0],[-1,0,0],[0,0,1]]
    Eigen::Matrix3d R_imu2body;
    R_imu2body << 0,  1,  0,
                 -1,  0,  0,
                  0,  0,  1;

    int frame = 0;
    while (g_running) {
        const float* q_raw = imu->get_quaternion();  // [w, x, y, z] WIT frame
        const float* g_raw = imu->get_gyro();        // [x, y, z] rad/s WIT frame
        const float* a_raw = imu->get_acc();         // [x, y, z] m/s^2 WIT frame

        // Convert quaternion to rotation matrix (WIT frame)
        Eigen::Vector4d q(q_raw[0], q_raw[1], q_raw[2], q_raw[3]);
        Eigen::Matrix3d R_wi = math_utils::quat_to_rotation(q);

        // Rotate to body frame
        Eigen::Matrix3d R_body = R_imu2body * R_wi;
        Eigen::Vector3d rpy    = math_utils::quat_to_euler(q);  // body frame (after R_imu2body applied)

        // Gravity direction in body frame (should be [0,0,-1] when flat)
        Eigen::Vector3d grav_world(0, 0, -1);
        Eigen::Vector3d grav_body = R_body.transpose() * grav_world;

        // Gyro in body frame
        Eigen::Vector3d gyro_imu(g_raw[0], g_raw[1], g_raw[2]);
        Eigen::Vector3d gyro_body = R_imu2body * gyro_imu;

        // Accel in body frame (for reference)
        Eigen::Vector3d accel_imu(a_raw[0], a_raw[1], a_raw[2]);
        Eigen::Vector3d accel_body = R_imu2body * accel_imu;

        // Print every 10 frames (~100ms)
        if (frame++ % 10 == 0) {
            std::cout << "\033[H\033[2J";  // clear terminal

            std::cout << "=== IMU Verification Tool === (Ctrl+C to stop)\n\n";

            std::cout << std::fixed << std::setprecision(4);

            std::cout << "--- Raw (WIT frame: X=right, Y=forward, Z=up) ---\n";
            std::cout << "  Quaternion [w,x,y,z]: "
                      << q_raw[0] << "  " << q_raw[1] << "  "
                      << q_raw[2] << "  " << q_raw[3] << "\n";
            std::cout << "  Gyro [x,y,z] rad/s:   "
                      << g_raw[0] << "  " << g_raw[1] << "  " << g_raw[2] << "\n";
            std::cout << "  Accel [x,y,z] m/s²:   "
                      << a_raw[0] << "  " << a_raw[1] << "  " << a_raw[2] << "\n\n";

            std::cout << "--- Body frame (after R_imu2body: X=forward, Y=left, Z=up) ---\n";
            std::cout << "  RPY [roll,pitch,yaw] rad:   "
                      << std::setw(8) << rpy[0] << "  "
                      << std::setw(8) << rpy[1] << "  "
                      << std::setw(8) << rpy[2] << "\n";
            std::cout << "  RPY [roll,pitch,yaw] deg:   "
                      << std::setw(8) << rpy[0]*180/M_PI << "  "
                      << std::setw(8) << rpy[1]*180/M_PI << "  "
                      << std::setw(8) << rpy[2]*180/M_PI << "\n";
            std::cout << "  Omega [x,y,z] rad/s:        "
                      << std::setw(8) << gyro_body[0] << "  "
                      << std::setw(8) << gyro_body[1] << "  "
                      << std::setw(8) << gyro_body[2] << "\n";
            std::cout << "  Gravity dir [x,y,z]:        "
                      << std::setw(8) << grav_body[0] << "  "
                      << std::setw(8) << grav_body[1] << "  "
                      << std::setw(8) << grav_body[2] << "\n\n";

            // Sanity checks
            std::cout << "--- Sanity checks (robot flat on ground, stationary) ---\n";

            auto check = [](const char* name, double val, double expect, double tol) {
                bool ok = std::abs(val - expect) < tol;
                std::cout << "  " << name << ": " << std::setw(8) << val
                          << "  (expect " << expect << ")  "
                          << (ok ? "✓" : "✗ MISMATCH") << "\n";
            };

            check("roll  (rad)", rpy[0],         0.0, 0.05);
            check("pitch (rad)", rpy[1],         0.0, 0.05);
            check("grav_x     ", grav_body[0],   0.0, 0.05);
            check("grav_y     ", grav_body[1],   0.0, 0.05);
            check("grav_z     ", grav_body[2],  -1.0, 0.05);
            check("omega_norm ", gyro_body.norm(), 0.0, 0.05);

            double qnorm = q.norm();
            std::cout << "  quat norm  : " << std::setw(8) << qnorm
                      << "  (expect 1.0)  " << (std::abs(qnorm - 1.0) < 0.01 ? "✓" : "✗") << "\n";

            std::cout << "\nTilt test: slowly tilt the robot and observe roll/pitch change.\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "\nDone.\n";
    return 0;
}
