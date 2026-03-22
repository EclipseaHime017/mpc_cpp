#pragma once
#include "common/robot_config.hpp"
#include "common/robot_state.hpp"
#include "kinematics/quadruped_kin.hpp"
#include <Eigen/Dense>

// Complementary filter state estimator
// Mirrors Python state_estimator.py, but replaces MuJoCo with analytical kinematics.
//
// IMU frame (WIT-Motion, from pure_cpp/observations.cpp):
//   IMU Y-axis = robot forward (body X)
//   IMU X-axis = robot right   (body -Y)
//   IMU Z-axis = robot up      (body Z)
// Quaternion from WIT SDK: [q0, q1, q2, q3] = [w, x, y, z] in IMU frame.
class StateEstimator {
public:
    StateEstimator(const RobotConfig& cfg, const QuadrupedKinematics& kin);

    // Update state from raw sensor data.
    // Call at 500Hz from PD thread.
    //
    // imu:        raw WIT IMU data
    // q:          joint angles (12-DOF, offset-subtracted, MPC ordering)
    // dq:         joint velocities
    // motor_tau:  estimated motor torques [Nm] (for contact detection)
    // dt:         control timestep [s]
    // time:       elapsed time since startup [s]
    void update(const IMUSensorData& imu,
                const Eigen::Matrix<double,12,1>& q,
                const Eigen::Matrix<double,12,1>& dq,
                const Eigen::Matrix<double,12,1>& motor_tau,
                double dt,
                double time);

    const RobotState& state() const { return state_; }

    // Reset estimator (called when robot is restarted)
    void reset();

private:
    const RobotConfig& cfg_;
    const QuadrupedKinematics& kin_;

    RobotState state_;

    // IMU bias (world frame, XY only)
    Eigen::Vector3d acc_bias_world_{0., 0., 0.};

    double last_time_ = -1.0;

    // Convert WIT IMU quaternion (IMU frame) -> body-to-world rotation matrix
    // IMU frame: Y=forward, X=right, Z=up
    // Body frame: X=forward, Y=left, Z=up
    // So body_q = imu_q composed with a fixed offset rotation
    Eigen::Matrix3d imu_quat_to_body_rot(const Eigen::Vector4d& q_imu) const;

    // Convert IMU angular velocity (IMU frame) to body frame
    // omega_body = R_imu_to_body * omega_imu
    Eigen::Vector3d imu_gyro_to_body(const Eigen::Vector3d& gyro_imu) const;
};
