#include "wbc/wbc_controller.hpp"
#include <cmath>

WBCController::WBCController(const RobotConfig& cfg, const QuadrupedKinematics& kin)
    : cfg_(cfg), kin_(kin) {}

Eigen::Matrix<double, 12, 1> WBCController::compute(const Input& input) {
    Eigen::Matrix<double, 12, 1> tau = Eigen::Matrix<double,12,1>::Zero();

    if (!input.state) return tau;
    const RobotState& state = *input.state;

    for (int i = 0; i < 4; ++i) {
        if (!input.contact[i]) continue;

        // Jacobian for leg i in body frame (3x3)
        Eigen::Matrix3d J_body = kin_.jacobian(i, state.q.segment<3>(i*3));

        // Transform to world frame: J_world = R * J_body
        Eigen::Matrix3d J_world = state.rot_mat * J_body;

        // Jacobian transpose mapping: tau_i = J_world^T * f_mpc_i
        // f_mpc is in world frame; J_world maps joint vel -> foot vel in world
        Eigen::Vector3d f_world = input.f_mpc.row(i).transpose();

        // Negate because MPC gives the force the ground exerts ON the foot;
        // the joint torque must produce an equal and opposite reaction.
        tau.segment<3>(i * 3) = J_world.transpose() * f_world;

        // Stance foot damping (suppress foot slipping velocity)
        Eigen::Vector3d v_foot_w = input.foot_vel_w.row(i).transpose();
        const double W_st = 5.0;  // damping gain (tunable)
        tau.segment<3>(i * 3) -= J_world.transpose() * (W_st * v_foot_w);
    }

    return tau;
}
