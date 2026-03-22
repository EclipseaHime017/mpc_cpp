#pragma once
#include "common/robot_config.hpp"
#include <Eigen/Dense>

// Analytical kinematics for a 3-DOF leg (HipA, HipF, Knee)
// Each leg: q = [hip_abduction, hip_flexion, knee] (offset-subtracted frame)
//
// Body frame convention (matches MuJoCo model and Python code):
//   X = forward, Y = left, Z = up
//
// Leg index: 0=LF, 1=LR, 2=RF, 3=RR
//   LF/LR: abduction sign = +1 (left side)
//   RF/RR: abduction sign = -1 (right side)
class QuadrupedKinematics {
public:
    explicit QuadrupedKinematics(const RobotConfig& cfg);

    // Forward kinematics: foot position in body frame for one leg
    // q_leg: [hip_abduction, hip_flexion, knee] (rad, offset-subtracted)
    Eigen::Vector3d fk_foot(int leg_idx, const Eigen::Vector3d& q_leg) const;

    // Analytical Jacobian dfoot_body/dq_leg (3x3)
    Eigen::Matrix3d jacobian(int leg_idx, const Eigen::Vector3d& q_leg) const;

    // Inverse kinematics (closed-form): body-frame foot position -> joint angles
    // Returns false if target is unreachable.
    bool ik_foot(int leg_idx, const Eigen::Vector3d& p_body,
                 Eigen::Vector3d& q_out) const;

    // Compute all 4 foot positions in body frame from full 12-DOF joint vector
    // q: [LF_HipA, LF_HipF, LF_Knee,  LR_..., RF_..., RR_...]
    Eigen::Matrix<double, 4, 3> fk_all(const Eigen::Matrix<double,12,1>& q) const;

    // Compute foot positions in world frame
    // q: 12-DOF joint vector, pos: base position, R: body->world rotation
    Eigen::Matrix<double, 4, 3> fk_world(const Eigen::Matrix<double,12,1>& q,
                                          const Eigen::Vector3d& pos,
                                          const Eigen::Matrix3d& R) const;

    // Foot velocity in world frame from joint velocities (Jacobian * dq)
    // dq: 12-DOF joint velocity, omega: base angular velocity (world frame)
    Eigen::Matrix<double, 4, 3> foot_vel_world(
        const Eigen::Matrix<double,12,1>& q,
        const Eigen::Matrix<double,12,1>& dq,
        const Eigen::Vector3d& vel_base,
        const Eigen::Vector3d& omega,
        const Eigen::Matrix3d& R) const;

private:
    const RobotConfig& cfg_;
    // Abduction sign: +1 for left (LF,LR), -1 for right (RF,RR)
    static constexpr double abduction_sign[4] = {1., 1., -1., -1.};
};
