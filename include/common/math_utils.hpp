#pragma once
#include <Eigen/Dense>
#include <cmath>

namespace math_utils {

// Quaternion [w, x, y, z] -> 3x3 rotation matrix (body -> world)
inline Eigen::Matrix3d quat_to_rotation(const Eigen::Vector4d& q) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    Eigen::Matrix3d R;
    R << 1 - 2*(y*y + z*z),   2*(x*y - w*z),     2*(x*z + w*y),
         2*(x*y + w*z),       1 - 2*(x*x + z*z), 2*(y*z - w*x),
         2*(x*z - w*y),       2*(y*z + w*x),     1 - 2*(x*x + y*y);
    return R;
}

// Quaternion [w, x, y, z] -> ZYX Euler angles [roll, pitch, yaw]
// Matches Python state_estimator._quat_to_euler()
inline Eigen::Vector3d quat_to_euler(const Eigen::Vector4d& q) {
    double w = q[0], x = q[1], y = q[2], z = q[3];
    double sinr_cosp = 2.0 * (w*x + y*z);
    double cosr_cosp = 1.0 - 2.0 * (x*x + y*y);
    double roll  = std::atan2(sinr_cosp, cosr_cosp);
    double sinp  = 2.0 * (w*y - z*x);
    double pitch = (std::abs(sinp) >= 1.0) ? std::copysign(M_PI/2.0, sinp) : std::asin(sinp);
    double siny_cosp = 2.0 * (w*z + x*y);
    double cosy_cosp = 1.0 - 2.0 * (y*y + z*z);
    double yaw   = std::atan2(siny_cosp, cosy_cosp);
    return {roll, pitch, yaw};
}

// 3x3 skew-symmetric matrix from 3D vector (for cross product: skew(a)*b = a x b)
inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<    0, -v[2],  v[1],
         v[2],     0, -v[0],
        -v[1],  v[0],     0;
    return S;
}

// Yaw-only rotation matrix: R_z(yaw)
inline Eigen::Matrix3d rot_z(double yaw) {
    double c = std::cos(yaw), s = std::sin(yaw);
    Eigen::Matrix3d R;
    R << c, -s, 0,
         s,  c, 0,
         0,  0, 1;
    return R;
}

// Clamp scalar
template<typename T>
inline T clamp(T val, T lo, T hi) {
    return (val < lo) ? lo : (val > hi) ? hi : val;
}

} // namespace math_utils
