#pragma once
#include "common/robot_config.hpp"
#include "common/robot_state.hpp"
#include <Eigen/Dense>
#include <array>

struct GaitOutput {
    std::array<bool, 4> contact;
    Eigen::Matrix<double, 4, 3> foot_target_pos;  // world frame
    Eigen::Matrix<double, 4, 3> foot_target_vel;  // world frame
};

class GaitGenerator {
public:
    explicit GaitGenerator(const RobotConfig& cfg);

    // Advance gait by dt, compute contact state and swing targets.
    // Call at 500Hz.
    GaitOutput update(double dt, const RobotState& state,
                      const Eigen::Vector3d& v_cmd_world,
                      const Eigen::Matrix<double,4,3>& nmpc_foot_target);

    // Build MPC reference trajectory for horizon steps.
    // v_cmd_body: velocity command in body frame
    // Returns [horizon x 12] reference state matrix (row-major)
    Eigen::MatrixXd get_mpc_reference(const RobotState& state,
                                       double target_z,
                                       const Eigen::Vector3d& v_cmd_body,
                                       double yaw_rate_cmd) const;

    // Predict contact sequence for next horizon steps (used by NMPC)
    // Returns [4 x horizon] matrix
    Eigen::MatrixXd predict_contact_sequence(int horizon) const;

    double phase() const { return phase_; }
    void   set_phase(double p) { phase_ = p; }
    const std::array<double,4>& phase_offsets() const { return cfg_.phase_offsets; }
    const Eigen::Matrix<double,4,3>& swing_start_pos() const { return swing_start_pos_; }

private:
    const RobotConfig& cfg_;

    double phase_ = 0.0;
    Eigen::Matrix<double, 4, 3> swing_start_pos_;
    std::array<bool, 4> prev_contact_{true, true, true, true};

    // Bezier-like swing trajectory (matches Python _get_swing_trajectory)
    std::pair<Eigen::Vector3d, Eigen::Vector3d>
    swing_trajectory(double swing_phase,
                     const Eigen::Vector3d& start_p,
                     const Eigen::Vector3d& end_p) const;
};
