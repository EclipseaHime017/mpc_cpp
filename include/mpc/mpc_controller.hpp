#pragma once
#include "common/robot_config.hpp"
#include "common/robot_state.hpp"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <array>

// Convex MPC Controller (direct port of mpc_controller.py)
// Solves a QP with OSQP at 30Hz to compute ground reaction forces.
//
// Key design decisions (matching Python fixes):
//   Fix 1: B matrix structure always built for all 4 legs; contact enforced via bounds
//   Fix 2: X_ref is NOT transposed (flatten row-major)
//   Fix 3: OSQP solver is re-initialized every call for numerical stability
class MPCController {
public:
    explicit MPCController(const RobotConfig& cfg);

    // Solve for optimal GRF.
    // contact_state: which legs are in contact (must match Python logic)
    // desired_traj:  [horizon x 12] reference state (row = one timestep)
    // Returns: 12-dim GRF vector [f_LF, f_LR, f_RF, f_RR] (world frame, N)
    // On solver failure, returns last valid result.
    Eigen::Matrix<double, 12, 1> solve(
        const RobotState& state,
        const Eigen::Matrix<double, 4, 3>& foot_positions,
        const std::array<bool, 4>& contact_state,
        const Eigen::MatrixXd& desired_traj);

    double last_solve_ms() const { return last_solve_ms_; }

private:
    const RobotConfig& cfg_;

    Eigen::Matrix<double, 12, 1> last_result_ = Eigen::Matrix<double,12,1>::Zero();
    double last_solve_ms_ = 0.0;

    // Build discrete-time system matrices (A_d, B_d, g_d) for current state
    void build_system(const RobotState& state,
                      const Eigen::Matrix<double,4,3>& foot_pos,
                      Eigen::MatrixXd& Ad,
                      Eigen::MatrixXd& Bd,
                      Eigen::VectorXd& gd) const;
};
