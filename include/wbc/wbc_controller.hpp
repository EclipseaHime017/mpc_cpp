#pragma once
#include "common/robot_config.hpp"
#include "common/robot_state.hpp"
#include "kinematics/quadruped_kin.hpp"
#include <Eigen/Dense>

// Simplified Whole-Body Controller (port of wbc_controller.py)
//
// Without MuJoCo, we use:
//   - Simplified floating-base dynamics (approximate M, h from analytical model)
//   - Analytical Jacobians from QuadrupedKinematics
//
// For the real robot, the primary control is:
//   tau = J^T * f_mpc  (Jacobian transpose for stance legs)
// plus PD error from the main loop.
//
// The full QP formulation is preserved as an optional path.
class WBCController {
public:
    WBCController(const RobotConfig& cfg, const QuadrupedKinematics& kin);

    struct Input {
        Eigen::Matrix<double, 4, 3> f_mpc;       // MPC desired GRF [N], world frame
        Eigen::Matrix<double, 4, 3> foot_vel_w;  // foot velocities, world frame
        Eigen::Matrix<double, 12, 1> dq;          // joint velocities
        std::array<bool, 4> contact;
        const RobotState* state = nullptr;        // full state (for Jacobians)
    };

    // Compute joint torques (12-DOF) to track f_mpc.
    // Uses Jacobian transpose method (fast, suitable for 500Hz).
    // tau_out: [12] joint torques (offset-subtracted ordering)
    Eigen::Matrix<double, 12, 1> compute(const Input& input);

private:
    const RobotConfig& cfg_;
    const QuadrupedKinematics& kin_;
};
