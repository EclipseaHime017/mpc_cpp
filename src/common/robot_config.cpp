#include "common/robot_config.hpp"
#include <yaml-cpp/yaml.h>
#include <stdexcept>
#include <iostream>

RobotConfig::RobotConfig() {
    // Defaults for Eigen members that can't be set inline
    I_body = Eigen::DiagonalMatrix<double,3>(0.1, 0.25, 0.25).toDenseMatrix();
    com_offset << 0.003, 0.0, 0.058;

    hip_offsets <<
         0.16,  0.0675, 0.0,
        -0.16,  0.0675, 0.0,
         0.16, -0.0675, 0.0,
        -0.16, -0.0675, 0.0;

    weights_Q << 150., 150., 50., 0., 50., 150.,
                  20.,  20., 20., 10., 10., 10.;

    // Joint offsets: LF_HipA, LR_HipA, RF_HipA, RR_HipA,
    //                LF_HipF, LR_HipF, RF_HipF, RR_HipF,
    //                LF_Knee, LR_Knee, RF_Knee, RR_Knee
    joint_offsets <<  0.37, -0.37, -0.37,  0.37,
                      0.13,  0.13, -0.13, -0.13,
                  1.7681, 1.7681, -1.7681, -1.7681;
}

static double read_double(const YAML::Node& n) { return n.as<double>(); }

RobotConfig RobotConfig::from_yaml(const std::string& path) {
    RobotConfig cfg;  // start with defaults
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const std::exception& e) {
        std::cerr << "[RobotConfig] Failed to load " << path << ": " << e.what()
                  << " — using defaults\n";
        return cfg;
    }

    // Helper to read optional scalar
    auto opt_d = [&](const YAML::Node& n, const std::string& k, double& out) {
        if (n[k]) out = n[k].as<double>();
    };
    auto opt_i = [&](const YAML::Node& n, const std::string& k, int& out) {
        if (n[k]) out = n[k].as<int>();
    };

    if (auto rb = root["robot"]) {
        opt_d(rb, "mass", cfg.mass);
        if (rb["I_body"]) {
            auto v = rb["I_body"].as<std::vector<double>>();
            if (v.size() == 9)
                cfg.I_body = Eigen::Map<Eigen::Matrix3d>(v.data()).transpose();
        }
        if (rb["com_offset"]) {
            auto v = rb["com_offset"].as<std::vector<double>>();
            cfg.com_offset << v[0], v[1], v[2];
        }
        opt_d(rb, "L1", cfg.L1);
        opt_d(rb, "L2", cfg.L2);
        opt_d(rb, "L3", cfg.L3);
        if (rb["hip_offsets"] && rb["hip_offsets"].IsSequence()) {
            for (int i = 0; i < 4 && i < (int)rb["hip_offsets"].size(); ++i) {
                auto v = rb["hip_offsets"][i].as<std::vector<double>>();
                cfg.hip_offsets.row(i) << v[0], v[1], v[2];
            }
        }
        opt_d(rb, "target_z", cfg.target_z);
    }

    if (auto g = root["gait"]) {
        opt_d(g, "period",       cfg.gait_period);
        opt_d(g, "duty_factor",  cfg.duty_factor);
        opt_d(g, "step_height",  cfg.step_height);
        if (g["phase_offsets"]) {
            auto v = g["phase_offsets"].as<std::vector<double>>();
            for (int i = 0; i < 4 && i < (int)v.size(); ++i)
                cfg.phase_offsets[i] = v[i];
        }
    }

    if (auto m = root["mpc"]) {
        opt_d(m, "freq",           cfg.mpc_freq);
        opt_i(m, "horizon",        cfg.mpc_horizon);
        opt_d(m, "mu",             cfg.mu);
        opt_d(m, "f_min",          cfg.f_min);
        opt_d(m, "f_max",          cfg.f_max);
        opt_d(m, "weights_R",      cfg.weights_R);
        opt_i(m, "ipopt_max_iter", cfg.ipopt_max_iter);
        opt_d(m, "ipopt_tol",      cfg.ipopt_tol);
        if (m["weights_Q"]) {
            auto v = m["weights_Q"].as<std::vector<double>>();
            for (int i = 0; i < 12 && i < (int)v.size(); ++i)
                cfg.weights_Q[i] = v[i];
        }
    }

    if (auto c = root["control"]) {
        opt_d(c, "kp_stand",   cfg.kp_stand);
        opt_d(c, "kd_stand",   cfg.kd_stand);
        opt_d(c, "kp_swing",   cfg.kp_swing);
        opt_d(c, "kd_swing",   cfg.kd_swing);
        opt_d(c, "kp_stance",  cfg.kp_stance);
        opt_d(c, "kd_stance",  cfg.kd_stance);
        opt_d(c, "torque_limit",cfg.torque_limit);
        opt_d(c, "nmpc_alpha", cfg.nmpc_alpha);
        opt_d(c, "vcmd_alpha", cfg.vcmd_alpha);
    }

    if (auto hw = root["hardware"]) {
        if (hw["can_interfaces"]) {
            cfg.can_interfaces = hw["can_interfaces"].as<std::vector<std::string>>();
        }
        if (hw["imu_device"])   cfg.imu_device  = hw["imu_device"].as<std::string>();
        if (hw["gamepad_device"])cfg.gamepad_dev = hw["gamepad_device"].as<std::string>();
        if (hw["motor_ids"]) {
            auto v = hw["motor_ids"].as<std::vector<int>>();
            for (int i = 0; i < 12 && i < (int)v.size(); ++i)
                cfg.motor_ids[i] = v[i];
        }
        opt_d(hw, "mit_kp",           cfg.mit_kp);
        opt_d(hw, "mit_kd",           cfg.mit_kd);
        opt_d(hw, "mit_vel_limit",    cfg.mit_vel_limit);
        opt_d(hw, "mit_torque_limit", cfg.mit_torque_limit);
        opt_d(hw, "knee_gear_ratio",  cfg.knee_gear_ratio);
        opt_d(hw, "contact_torque_threshold", cfg.contact_torque_threshold);
        if (hw["joint_offsets"]) {
            auto v = hw["joint_offsets"].as<std::vector<double>>();
            for (int i = 0; i < 12 && i < (int)v.size(); ++i)
                cfg.joint_offsets[i] = v[i];
        }
    }

    if (auto e = root["estimator"]) {
        opt_d(e, "alpha_v",        cfg.alpha_v);
        opt_d(e, "alpha_z",        cfg.alpha_z);
        opt_d(e, "alpha_bias",     cfg.alpha_bias);
        opt_d(e, "calib_duration", cfg.calib_duration);
    }

    return cfg;
}
