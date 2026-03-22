#include "kinematics/quadruped_kin.hpp"
#include <cmath>
#include <stdexcept>

constexpr double QuadrupedKinematics::abduction_sign[4];

QuadrupedKinematics::QuadrupedKinematics(const RobotConfig& cfg) : cfg_(cfg) {}

Eigen::Vector3d QuadrupedKinematics::fk_foot(int leg_idx, const Eigen::Vector3d& q_leg) const {
    // q_leg: [q1=abduction, q2=flexion, q3=knee]
    double q1 = q_leg[0], q2 = q_leg[1], q3 = q_leg[2];
    double L1 = cfg_.L1, L2 = cfg_.L2, L3 = cfg_.L3;
    double s  = abduction_sign[leg_idx];  // +1 left, -1 right
    double c1 = std::cos(q1), s1 = std::sin(q1);
    double c2 = std::cos(q2), s2 = std::sin(q2);
    double c23 = std::cos(q2 + q3), s23 = std::sin(q2 + q3);

    // Three-link leg kinematics (standard DH-like, body frame)
    // Hip offset is handled separately via hip_offsets; here we return
    // the foot position relative to the hip joint.
    double px = L2 * s2 + L3 * s23;
    double py = s * L1 * c1 + (L2 * c2 + L3 * c23) * s1;
    double pz = -s * L1 * s1 + (L2 * c2 + L3 * c23) * c1;

    // Return position relative to hip, then caller adds hip_offsets
    return Eigen::Vector3d(px, py, pz) + cfg_.hip_offsets.row(leg_idx).transpose();
}

Eigen::Matrix3d QuadrupedKinematics::jacobian(int leg_idx, const Eigen::Vector3d& q_leg) const {
    double q1 = q_leg[0], q2 = q_leg[1], q3 = q_leg[2];
    double L1 = cfg_.L1, L2 = cfg_.L2, L3 = cfg_.L3;
    double s  = abduction_sign[leg_idx];
    double c1 = std::cos(q1), s1 = std::sin(q1);
    double c2 = std::cos(q2), s2 = std::sin(q2);
    double c23 = std::cos(q2 + q3), s23 = std::sin(q2 + q3);
    double lc2 = L2 * c2 + L3 * c23;   // thigh+shank horizontal reach
    double ls2 = L2 * s2 + L3 * s23;   // thigh+shank vertical reach

    // dx/dqi
    double dxdq1 = 0.0;
    double dxdq2 = L2 * c2 + L3 * c23;
    double dxdq3 = L3 * c23;

    // dy/dqi
    double dydq1 = -s * L1 * s1 + lc2 * c1;
    double dydq2 = (-L2 * s2 - L3 * s23) * s1;
    double dydq3 = -L3 * s23 * s1;

    // dz/dqi
    double dzdq1 = -s * L1 * c1 - lc2 * s1;
    double dzdq2 = (-L2 * s2 - L3 * s23) * c1;
    double dzdq3 = -L3 * s23 * c1;

    Eigen::Matrix3d J;
    J << dxdq1, dxdq2, dxdq3,
         dydq1, dydq2, dydq3,
         dzdq1, dzdq2, dzdq3;
    return J;
}

bool QuadrupedKinematics::ik_foot(int leg_idx, const Eigen::Vector3d& p_body,
                                   Eigen::Vector3d& q_out) const {
    // p_body: foot position in body frame
    // Subtract hip offset to get position relative to hip joint
    Eigen::Vector3d p = p_body - cfg_.hip_offsets.row(leg_idx).transpose();
    double px = p[0], py = p[1], pz = p[2];
    double L1 = cfg_.L1, L2 = cfg_.L2, L3 = cfg_.L3;
    double s = abduction_sign[leg_idx];

    // Solve q1 (abduction)
    // From FK: py = s*L1*cos(q1) + lc2*sin(q1)
    //          pz = -s*L1*sin(q1) + lc2*cos(q1)
    // => [py,pz] = R(q1)*[s*L1, lc2], so angle in yz plane:
    //    atan2(py,pz) = q1 + atan2(s*L1, lc2) = q1 + gamma
    // => q1 = atan2(py,pz) - gamma,  where lc2 = sqrt(r_yz^2 - L1^2)
    double r_yz = std::sqrt(py*py + pz*pz);
    if (r_yz < L1 * 0.99) return false;  // unreachable
    double lc2_len = std::sqrt(r_yz*r_yz - L1*L1);
    double alpha = std::atan2(py, pz);
    double gamma = std::atan2(s * L1, lc2_len);
    double q1 = alpha - gamma;

    // After resolving q1, compute the remaining 2D reach
    // The remaining link starts along the y-z plane after hip rotation
    double pz_after = std::sqrt(r_yz*r_yz - L1*L1);  // effective z distance
    double D = std::sqrt(px*px + pz_after*pz_after);  // 2D distance to foot

    if (D > L2 + L3 || D < std::abs(L2 - L3)) return false;

    // Solve q3 (knee, using cosine rule)
    double cos_q3 = (D*D - L2*L2 - L3*L3) / (2.0 * L2 * L3);
    cos_q3 = std::max(-1.0, std::min(1.0, cos_q3));
    double q3 = -std::acos(cos_q3);  // knee bends backwards -> negative

    // Solve q2 (hip flexion)
    // From FK: [px; pz_after] = L2*[sin(q2); cos(q2)] + L3*[sin(q2+q3); cos(q2+q3)]
    // => atan2(px, pz_after) = q2 + atan2(L3*sin(q3), L2+L3*cos(q3))
    double q2 = std::atan2(px, pz_after)
              - std::atan2(L3 * std::sin(q3), L2 + L3 * std::cos(q3));

    q_out = {q1, q2, q3};
    return true;
}

Eigen::Matrix<double, 4, 3> QuadrupedKinematics::fk_all(
    const Eigen::Matrix<double,12,1>& q) const
{
    Eigen::Matrix<double, 4, 3> result;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d q_leg = q.segment<3>(i * 3);
        result.row(i) = fk_foot(i, q_leg).transpose();
    }
    return result;
}

Eigen::Matrix<double, 4, 3> QuadrupedKinematics::fk_world(
    const Eigen::Matrix<double,12,1>& q,
    const Eigen::Vector3d& pos,
    const Eigen::Matrix3d& R) const
{
    Eigen::Matrix<double, 4, 3> result;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d p_body = fk_foot(i, q.segment<3>(i * 3));
        result.row(i) = (pos + R * p_body).transpose();
    }
    return result;
}

Eigen::Matrix<double, 4, 3> QuadrupedKinematics::foot_vel_world(
    const Eigen::Matrix<double,12,1>& q,
    const Eigen::Matrix<double,12,1>& dq,
    const Eigen::Vector3d& vel_base,
    const Eigen::Vector3d& omega,
    const Eigen::Matrix3d& R) const
{
    Eigen::Matrix<double, 4, 3> result;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d q_leg  = q.segment<3>(i * 3);
        Eigen::Vector3d dq_leg = dq.segment<3>(i * 3);
        Eigen::Matrix3d J_body  = jacobian(i, q_leg);
        // foot position relative to base in world frame
        Eigen::Vector3d p_body = fk_foot(i, q_leg);
        Eigen::Vector3d p_rel  = R * p_body;  // foot relative to base, world frame
        // v_foot = v_base + omega x p_rel + R * J_body * dq_leg
        Eigen::Vector3d v_joint = R * J_body * dq_leg;
        result.row(i) = (vel_base + omega.cross(p_rel) + v_joint).transpose();
    }
    return result;
}
