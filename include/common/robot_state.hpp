#pragma once
#include <Eigen/Dense>
#include <array>

// Complete robot state, computed by StateEstimator at 500Hz
struct RobotState {
    Eigen::Vector3d rpy;            // [roll, pitch, yaw] ZYX Euler (rad)
    Eigen::Vector3d pos;            // [px, py, pz] world frame (m)
    Eigen::Vector3d omega;          // angular velocity, world frame (rad/s)
    Eigen::Vector3d vel;            // linear velocity, world frame (m/s)
    Eigen::Matrix3d rot_mat;        // rotation matrix: body -> world

    // Foot positions and velocities in world frame (4 legs: LF, LR, RF, RR)
    Eigen::Matrix<double, 4, 3> foot_pos_world;
    Eigen::Matrix<double, 4, 3> foot_vel_world;
    // Foot positions in body frame
    Eigen::Matrix<double, 4, 3> foot_pos_body;

    // Joint angles and velocities (12 DOF, offset-subtracted)
    // Order: [LF_HipA, LF_HipF, LF_Knee,
    //         LR_HipA, LR_HipF, LR_Knee,
    //         RF_HipA, RF_HipF, RF_Knee,
    //         RR_HipA, RR_HipF, RR_Knee]
    Eigen::Matrix<double, 12, 1> q;
    Eigen::Matrix<double, 12, 1> dq;

    // Contact flags (true = foot in contact with ground)
    std::array<bool, 4> contact{true, true, true, true};

    double timestamp = 0.0;  // seconds since startup

    RobotState() {
        rpy.setZero(); pos.setZero(); omega.setZero(); vel.setZero();
        rot_mat.setIdentity();
        foot_pos_world.setZero(); foot_vel_world.setZero(); foot_pos_body.setZero();
        q.setZero(); dq.setZero();
    }
};

// MPC output cache: updated by MPC thread at 30Hz, read by PD thread at 500Hz
struct MPCOutput {
    // Ground reaction forces for each leg (LF, LR, RF, RR), world frame (N)
    Eigen::Matrix<double, 4, 3> f_stance = Eigen::Matrix<double,4,3>::Zero();
    // Optimized footstep placement (x, y) in world frame, per leg
    Eigen::Matrix<double, 4, 2> foot_placement = Eigen::Matrix<double,4,2>::Zero();
    double solve_time_ms = 0.0;
    bool valid = false;
};

// IMU sensor data (raw, read from WIT SDK registers)
struct IMUSensorData {
    // Quaternion [w, x, y, z] from WIT SDK (sReg[q0..q3] / 32768.0)
    Eigen::Vector4d quat{1.0, 0.0, 0.0, 0.0};
    // Gyroscope (rad/s), in IMU frame
    // IMU frame: Y=forward, X=right, Z=up (see observations.cpp)
    Eigen::Vector3d gyro_imu;
    // Accelerometer (m/s^2), in IMU frame, without gravity subtraction
    Eigen::Vector3d accel_imu;
};

// Joystick / keyboard command
struct CommandInput {
    double vx  = 0.0;    // forward velocity command (m/s)
    double vy  = 0.0;    // lateral velocity command (m/s)
    double yaw_rate = 0.0;  // yaw rate command (rad/s)
    bool   stop = false;
};
