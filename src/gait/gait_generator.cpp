#include "gait/gait_generator.hpp"
#include "common/math_utils.hpp"
#include <cmath>
#include <algorithm>

GaitGenerator::GaitGenerator(const RobotConfig& cfg) : cfg_(cfg) {
    swing_start_pos_.setZero();
}

GaitOutput GaitGenerator::update(double dt, const RobotState& state,
                                  const Eigen::Vector3d& v_cmd_world,
                                  const Eigen::Matrix<double,4,3>& nmpc_foot_target)
{
    phase_ = std::fmod(phase_ + dt / cfg_.gait_period, 1.0);

    GaitOutput out;
    out.foot_target_pos.setZero();
    out.foot_target_vel.setZero();

    const Eigen::Vector3d& base_pos = state.pos;
    const Eigen::Vector3d& base_vel = state.vel;
    const Eigen::Matrix3d& R        = state.rot_mat;
    double T_stance = cfg_.gait_period * cfg_.duty_factor;

    for (int i = 0; i < 4; ++i) {
        double leg_phase = std::fmod(phase_ + cfg_.phase_offsets[i], 1.0);

        if (leg_phase < cfg_.duty_factor) {
            // --- Stance phase ---
            out.contact[i] = true;
            prev_contact_[i] = true;
        } else {
            // --- Swing phase ---
            out.contact[i] = false;
            double swing_phase = (leg_phase - cfg_.duty_factor) / (1.0 - cfg_.duty_factor);

            // Record swing start position on contact->swing transition
            if (prev_contact_[i]) {
                swing_start_pos_.row(i) = state.foot_pos_world.row(i);
            }
            prev_contact_[i] = false;

            // Target: NMPC-provided footstep (x,y) + z=0
            Eigen::Vector3d target;
            target[0] = nmpc_foot_target(i, 0);
            target[1] = nmpc_foot_target(i, 1);
            target[2] = 0.0;

            auto [p_des, v_des] = swing_trajectory(swing_phase,
                                                    swing_start_pos_.row(i).transpose(),
                                                    target);
            out.foot_target_pos.row(i) = p_des.transpose();
            out.foot_target_vel.row(i) = v_des.transpose();
        }
    }
    return out;
}

std::pair<Eigen::Vector3d, Eigen::Vector3d>
GaitGenerator::swing_trajectory(double phase,
                                 const Eigen::Vector3d& start_p,
                                 const Eigen::Vector3d& end_p) const
{
    Eigen::Vector3d p, v;
    double t_swing = cfg_.gait_period * (1.0 - cfg_.duty_factor);

    // XY: linear interpolation
    p.head<2>() = start_p.head<2>() + phase * (end_p.head<2>() - start_p.head<2>());
    v.head<2>() = (end_p.head<2>() - start_p.head<2>()) / t_swing;

    // Z: sinusoidal arc
    double z_start = std::max(0.0, start_p[2]);
    p[2] = z_start * (1.0 - phase) + cfg_.step_height * std::sin(phase * M_PI);
    v[2] = -z_start / t_swing + cfg_.step_height * M_PI * std::cos(phase * M_PI) / t_swing;

    return {p, v};
}

Eigen::MatrixXd GaitGenerator::get_mpc_reference(const RobotState& state,
                                                   double target_z,
                                                   const Eigen::Vector3d& v_cmd_body,
                                                   double yaw_rate_cmd) const
{
    int    N  = cfg_.mpc_horizon;
    double dt = 1.0 / cfg_.mpc_freq;

    // Convert body velocity command to world frame
    double yaw = state.rpy[2];
    double cy  = std::cos(yaw), sy = std::sin(yaw);
    Eigen::Vector3d v_world{
        v_cmd_body[0] * cy - v_cmd_body[1] * sy,
        v_cmd_body[0] * sy + v_cmd_body[1] * cy,
        0.0
    };

    // X_ref: [horizon x 12], state = [roll, pitch, yaw, px, py, pz, ox, oy, oz, vx, vy, vz]
    Eigen::MatrixXd X_ref(N, 12);
    for (int k = 0; k < N; ++k) {
        X_ref(k, 0) = 0.0;                                // roll ref
        X_ref(k, 1) = 0.0;                                // pitch ref
        X_ref(k, 2) = state.rpy[2] + yaw_rate_cmd * (k+1) * dt;  // yaw ref
        X_ref(k, 3) = state.pos[0];                        // px (weight=0 in Q)
        X_ref(k, 4) = state.pos[1];                        // py (weight=0 in Q)
        X_ref(k, 5) = target_z;                            // pz
        X_ref(k, 6) = 0.0;                                // ox
        X_ref(k, 7) = 0.0;                                // oy
        X_ref(k, 8) = yaw_rate_cmd;                        // oz
        X_ref.block<1,3>(k, 9) = v_world.transpose();      // vx, vy, vz
    }
    return X_ref;
}

Eigen::MatrixXd GaitGenerator::predict_contact_sequence(int horizon) const {
    Eigen::MatrixXd contact_seq(4, horizon);
    double dt = 1.0 / cfg_.mpc_freq;

    for (int k = 0; k < horizon; ++k) {
        double t_pred = k * dt;
        for (int i = 0; i < 4; ++i) {
            double leg_phase = std::fmod(phase_ + t_pred / cfg_.gait_period
                                         + cfg_.phase_offsets[i], 1.0);
            contact_seq(i, k) = (leg_phase < cfg_.duty_factor) ? 1.0 : 0.0;
        }
    }
    return contact_seq;
}
