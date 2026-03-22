#pragma once
#include "common/robot_config.hpp"
#include "common/robot_state.hpp"
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <chrono>

// Nonlinear MPC Footstep Planner (direct port of nmpc_planner.py)
// Uses CasADi + IPOPT to jointly optimize GRF and footstep placement.
//
// NLP is built once in the constructor; each solve() only does numerical evaluation.
// Warm-starting from the previous solution.
class NMPCFootstepPlanner {
public:
    explicit NMPCFootstepPlanner(const RobotConfig& cfg);

    struct Result {
        Eigen::Matrix<double, 12, 1> f_first_step;    // GRF at k=0 [N]
        Eigen::Matrix<double, 4, 2>  foot_placement;  // optimal footstep (x,y) world frame
        double solve_time_ms = 0.0;
        bool   converged     = false;
    };

    // contact_seq:        [4 x horizon] (1=stance, 0=swing)
    // foot_pos_curr:      [4 x 3] current foot positions in world frame
    // X_ref:              [horizon x 12] reference trajectory
    // hip_offsets:        [4 x 3] hip positions in body frame
    // nominal_foot_offsets: [4 x 3] default foot positions in body frame
    Result solve(const RobotState& state,
                 const Eigen::MatrixXd& contact_seq,
                 const Eigen::Matrix<double,4,3>& foot_pos_curr,
                 const Eigen::MatrixXd& X_ref,
                 const Eigen::Matrix<double,4,3>& hip_offsets,
                 const Eigen::Matrix<double,4,3>& nominal_foot_offsets);

    double last_solve_ms() const { return last_solve_ms_; }

private:
    void build_nlp();  // called once in constructor

    const RobotConfig& cfg_;

    // CasADi NLP function (compiled once)
    casadi::Function nlp_solver_;

    // Warm-start cache
    casadi::DM warm_U_;   // [12 x horizon]
    casadi::DM warm_P_;   // [4 x 2]

    // Constraint bounds (built once)
    casadi::DM g_lb_, g_ub_;

    int n_vars_    = 0;
    int n_cons_    = 0;
    double last_solve_ms_ = 0.0;
};
