#include "estimator/state_estimator.hpp"
#include "common/math_utils.hpp"
#include <cmath>
#include <algorithm>

StateEstimator::StateEstimator(const RobotConfig& cfg, const QuadrupedKinematics& kin)
    : cfg_(cfg), kin_(kin)
{
    reset();
}

void StateEstimator::reset() {
    state_ = RobotState{};
    state_.pos[2] = cfg_.target_z;
    state_.rot_mat.setIdentity();
    acc_bias_world_.setZero();
    last_time_ = -1.0;
}

// IMU frame (WIT-Motion): X=right, Y=forward, Z=up
// Body frame (MPC):       X=forward, Y=left, Z=up
// Rotation from IMU -> body:
//   body_x =  imu_y  (forward)
//   body_y = -imu_x  (left = -right)
//   body_z =  imu_z  (up)
// This is a rotation about Z by -90 deg:
//   R_imu2body = [[0,  1, 0],
//                 [-1, 0, 0],
//                 [0,  0, 1]]
static const Eigen::Matrix3d R_imu2body = (Eigen::Matrix3d() <<
     0,  1, 0,
    -1,  0, 0,
     0,  0, 1).finished();

Eigen::Matrix3d StateEstimator::imu_quat_to_body_rot(const Eigen::Vector4d& q_imu) const {
    // Rotation from WIT quaternion is "world->IMU" or "IMU->world"?
    // WIT quaternion represents body orientation in world frame (IMU frame axes).
    // R_imu_world: rotates world vectors into the IMU frame.
    // Actually WIT quaternion = R that rotates body(IMU) axes in world:
    //   R_world_imu = quat_to_rotation(q_imu)
    // We want R_world_body = R_world_imu * R_imu_body
    //                      = R_world_imu * R_imu2body^T
    Eigen::Matrix3d R_world_imu = math_utils::quat_to_rotation(q_imu);
    return R_world_imu * R_imu2body.transpose();
}

Eigen::Vector3d StateEstimator::imu_gyro_to_body(const Eigen::Vector3d& gyro_imu) const {
    // gyro_imu is in IMU frame (deg/s, already converted to rad/s by caller)
    // body frame:
    //   body_omega_x =  gyro_imu_y  (forward)
    //   body_omega_y = -gyro_imu_x  (left)
    //   body_omega_z =  gyro_imu_z  (up)
    return R_imu2body * gyro_imu;
}

void StateEstimator::update(const IMUSensorData& imu,
                             const Eigen::Matrix<double,12,1>& q,
                             const Eigen::Matrix<double,12,1>& dq,
                             const Eigen::Matrix<double,12,1>& motor_tau,
                             double dt,
                             double time)
{
    if (last_time_ < 0.0) {
        state_.q  = q;
        state_.dq = dq;
        last_time_ = time;
    }
    state_.q  = q;
    state_.dq = dq;
    state_.timestamp = time;

    // ========== 1. IMU -> body frame ==========
    state_.rot_mat = imu_quat_to_body_rot(imu.quat);
    state_.rpy     = math_utils::quat_to_euler(imu.quat);
    // Map euler angles: the WIT quaternion is in IMU frame, which has Y=forward.
    // After coordinate transform, rpy should be in body (X=forward) frame.
    // For simplicity, we re-derive RPY from the body rotation matrix.
    {
        const auto& R = state_.rot_mat;
        state_.rpy[0] = std::atan2(R(2,1), R(2,2));           // roll
        state_.rpy[1] = std::atan2(-R(2,0),
                                    std::sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));  // pitch
        state_.rpy[2] = std::atan2(R(1,0), R(0,0));           // yaw
    }

    // IMU angular velocity in body frame (world-frame omega = R * omega_body)
    Eigen::Vector3d omega_body = imu_gyro_to_body(imu.gyro_imu);
    state_.omega = state_.rot_mat * omega_body;

    // IMU accelerometer in body frame -> world frame (subtract gravity)
    Eigen::Vector3d accel_body  = R_imu2body * imu.accel_imu;
    Eigen::Vector3d acc_world   = state_.rot_mat * accel_body + Eigen::Vector3d(0, 0, -9.81);

    // ========== 2. IMU bias calibration (first calib_duration seconds) ==========
    if (time < cfg_.calib_duration) {
        acc_bias_world_[0] = (1.0 - cfg_.alpha_bias) * acc_bias_world_[0]
                            + cfg_.alpha_bias * acc_world[0];
        acc_bias_world_[1] = (1.0 - cfg_.alpha_bias) * acc_bias_world_[1]
                            + cfg_.alpha_bias * acc_world[1];
    }
    acc_world[0] -= acc_bias_world_[0];
    acc_world[1] -= acc_bias_world_[1];

    // ========== 3. Kinematic observer (from stance legs) ==========
    // Foot positions in body frame from analytical FK
    state_.foot_pos_body = kin_.fk_all(q);
    // Foot positions in world frame
    for (int i = 0; i < 4; ++i) {
        state_.foot_pos_world.row(i) =
            (state_.pos + state_.rot_mat * state_.foot_pos_body.row(i).transpose()).transpose();
    }

    // Contact detection via motor torque magnitude
    for (int i = 0; i < 4; ++i) {
        double tau_norm = 0.0;
        for (int j = 0; j < 3; ++j)
            tau_norm += std::abs(motor_tau[i*3 + j]);
        state_.contact[i] = (tau_norm > cfg_.contact_torque_threshold * 3.0);
    }

    // Kinematic velocity estimate from stance legs
    Eigen::Vector3d v_kin_avg  = Eigen::Vector3d::Zero();
    double          p_z_avg    = 0.0;
    int             n_contact  = 0;

    for (int i = 0; i < 4; ++i) {
        if (!state_.contact[i]) continue;
        Eigen::Vector3d p_body_i = state_.foot_pos_body.row(i).transpose();
        Eigen::Vector3d p_rel_w  = state_.rot_mat * p_body_i;  // rel to base, world frame

        // Foot velocity from joint Jacobian (body frame) -> world frame
        Eigen::Matrix3d J_body = kin_.jacobian(i, q.segment<3>(i*3));
        Eigen::Vector3d v_joint_w = state_.rot_mat * J_body * dq.segment<3>(i*3);

        // Stance foot is stationary -> infer base velocity
        // v_foot = v_base + omega x p_rel + v_joint = 0 (contact)
        Eigen::Vector3d v_base_est = -(state_.omega.cross(p_rel_w) + v_joint_w);
        v_kin_avg += v_base_est;
        p_z_avg   += -p_rel_w[2];  // foot at z=0 world -> base height = -rel_z
        ++n_contact;
    }

    // ========== 4. Complementary filter fusion ==========
    if (n_contact > 0) {
        v_kin_avg /= n_contact;
        p_z_avg   /= n_contact;

        double a_v = cfg_.alpha_v;
        double a_z = cfg_.alpha_z;
        state_.vel = (1.0 - a_v) * (state_.vel + acc_world * dt) + a_v * v_kin_avg;
        state_.pos[2] = (1.0 - a_z) * (state_.pos[2] + state_.vel[2] * dt) + a_z * p_z_avg;
    } else {
        state_.vel   += acc_world * dt;
        state_.pos[2] += state_.vel[2] * dt;
    }
    state_.pos[0] += state_.vel[0] * dt;
    state_.pos[1] += state_.vel[1] * dt;

    // ========== 5. ZUPT lock during calibration ==========
    if (time < cfg_.calib_duration) {
        state_.vel[0] = 0.0;
        state_.vel[1] = 0.0;
        state_.pos[0] = 0.0;
        state_.pos[1] = 0.0;
    }

    // ========== 6. Update foot velocities ==========
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d p_body_i = state_.foot_pos_body.row(i).transpose();
        Eigen::Vector3d p_rel_w  = state_.rot_mat * p_body_i;
        state_.foot_pos_world.row(i) = (state_.pos + p_rel_w).transpose();

        Eigen::Matrix3d J_body   = kin_.jacobian(i, q.segment<3>(i*3));
        Eigen::Vector3d v_joint_w = state_.rot_mat * J_body * dq.segment<3>(i*3);
        state_.foot_vel_world.row(i) =
            (state_.vel + state_.omega.cross(p_rel_w) + v_joint_w).transpose();
    }

    last_time_ = time;
}
