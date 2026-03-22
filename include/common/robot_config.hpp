#pragma once
#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>

struct RobotConfig {
    // ----- Physical parameters -----
    double mass = 12.0;
    Eigen::Matrix3d I_body;          // body inertia [kg*m^2]
    Eigen::Vector3d com_offset;      // CoM offset from base frame origin [m]

    double L1 = 0.0675;   // hip abduction link
    double L2 = 0.21;     // thigh
    double L3 = 0.24;     // shank

    // Hip joint positions in body frame: row i = (LF, LR, RF, RR)
    Eigen::Matrix<double, 4, 3> hip_offsets;

    // Nominal foot offsets in body frame (calibrated at startup t~2s)
    Eigen::Matrix<double, 4, 3> nominal_foot_offsets = Eigen::Matrix<double,4,3>::Zero();

    // Desired joint angles for stand pose (offset-subtracted frame, 0 = default)
    Eigen::Matrix<double, 12, 1> stand_joint_angles = Eigen::Matrix<double,12,1>::Zero();

    double target_z = 0.2;

    // ----- Gait -----
    double gait_period  = 0.4;
    double duty_factor  = 0.5;
    double step_height  = 0.06;
    std::array<double, 4> phase_offsets{0.0, 0.5, 0.5, 0.0}; // LF, LR, RF, RR

    // ----- MPC -----
    double mpc_freq   = 30.0;
    int    mpc_horizon = 10;
    double mu    = 0.5;
    double f_min = 0.0;
    double f_max = 150.0;
    Eigen::Matrix<double, 12, 1> weights_Q;
    double weights_R     = 1e-5;
    int    ipopt_max_iter = 30;
    double ipopt_tol      = 1e-2;

    // ----- Control gains -----
    double kp_stand  = 100.0, kd_stand  = 2.0;
    double kp_swing  = 150.0, kd_swing  = 8.0;
    double kp_stance = 30.0,  kd_stance = 2.0;
    double torque_limit = 40.0;
    double nmpc_alpha   = 0.2;   // NMPC footstep EMA weight (new value)
    double vcmd_alpha   = 0.05;  // velocity command EMA weight

    // ----- Hardware -----
    std::vector<std::string> can_interfaces{"candle0","candle1","candle2","candle3"};
    std::string imu_device   = "/dev/ttyCH341USB0";
    std::string gamepad_dev  = "/dev/input/js0";

    // Motor IDs, order matches joint_offsets:
    //   [LF_HipA, LR_HipA, RF_HipA, RR_HipA,
    //    LF_HipF, LR_HipF, RF_HipF, RR_HipF,
    //    LF_Knee, LR_Knee, RF_Knee, RR_Knee]
    std::array<int, 12> motor_ids{1,5,9,13, 2,6,10,14, 3,7,11,15};

    double mit_kp           = 40.0;
    double mit_kd           = 0.5;
    double mit_vel_limit    = 44.0;
    double mit_torque_limit = 17.0;
    double knee_gear_ratio  = 1.667;  // knee motor shaft / joint

    // Joint offsets (raw encoder value at the default stand pose)
    // robot_joint_angle = raw_motor_angle - joint_offset
    // Same index order as motor_ids
    Eigen::Matrix<double, 12, 1> joint_offsets;

    double contact_torque_threshold = 2.0;  // [Nm] for contact detection

    // ----- State estimator -----
    double alpha_v        = 0.02;   // velocity complementary filter
    double alpha_z        = 0.1;    // height complementary filter
    double alpha_bias     = 0.01;   // IMU bias low-pass filter
    double calib_duration = 2.0;    // [s] bias calibration period

    // Load all parameters from a YAML file
    static RobotConfig from_yaml(const std::string& path);

    // Default constructor sets eigen members to sane defaults
    RobotConfig();
};
