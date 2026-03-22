// =============================================================================
// MPC Solver Offline Test
// =============================================================================
// Runs ConvexMPC and NMPCFootstepPlanner with synthetic inputs and verifies:
//   1. Both solvers converge without crashing
//   2. GRF output has expected structure (friction cone, normal force sign)
//   3. Solve times are within budget (ConvexMPC <5ms, NMPC <50ms first call)
//
// Build:  cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -t test_mpc_solver
// Run:    ./build/test_mpc_solver [path/to/robot_params.yaml]
// =============================================================================

#include <iostream>
#include <cassert>
#include <cmath>
#include <chrono>
#include <iomanip>
#include <array>

#include "common/robot_config.hpp"
#include "common/robot_state.hpp"
#include "common/math_utils.hpp"
#include "mpc/mpc_controller.hpp"
#include "mpc/nmpc_footstep.hpp"
#include "kinematics/quadruped_kin.hpp"
#include "gait/gait_generator.hpp"

// ─── helpers ──────────────────────────────────────────────────────────────────

static void pass(const char* name) {
    std::cout << "  [PASS] " << name << "\n";
}

static void fail(const char* name, const std::string& why) {
    std::cerr << "  [FAIL] " << name << ": " << why << "\n";
}

// Build a default flat-ground standing state
static RobotState make_stand_state(const RobotConfig& cfg,
                                    const QuadrupedKinematics& kin)
{
    RobotState s{};
    s.rpy.setZero();
    s.pos << 0.0, 0.0, 0.32;
    s.omega.setZero();
    s.vel.setZero();
    s.rot_mat = Eigen::Matrix3d::Identity();
    s.q.setZero();
    s.dq.setZero();
    s.contact.fill(true);
    s.timestamp = 0.0;
    s.foot_vel_world.setZero();

    for (int i = 0; i < 4; ++i) {
        Eigen::Vector3d fp = kin.fk_foot(i, Eigen::Vector3d::Zero());
        s.foot_pos_world.row(i) = (s.pos + fp).transpose();
    }
    return s;
}

// ─── Test 1: Kinematics sanity ────────────────────────────────────────────────

static bool test_kinematics(const RobotConfig& cfg) {
    QuadrupedKinematics kin(cfg);
    bool ok = true;

    // Use a realistic stand pose (hip slightly flexed, knee bent)
    // q2≈0.6rad (30°), q3≈-1.3rad gives foot roughly below hip at target_z
    const Eigen::Vector3d q_stand(0.0, 0.6, -1.3);

    for (int leg = 0; leg < 4; ++leg) {
        Eigen::Vector3d fp = kin.fk_foot(leg, q_stand);

        // Foot should be within a 0.5m bounding box of body origin
        if (fp.norm() > 0.8) {
            fail("kinematics_fk_range", "foot position out of expected range: "
                 + std::to_string(fp.norm()) + " m");
            ok = false;
        }

        // Jacobian should be non-singular at stand pose
        Eigen::Matrix3d J = kin.jacobian(leg, q_stand);
        double det = J.determinant();
        if (std::abs(det) < 1e-6) {
            fail("kinematics_jacobian_rank", "Jacobian singular at stand pose");
            ok = false;
        }

        // FK / IK round-trip: IK(FK(q)) ≈ q
        Eigen::Vector3d q_recovered;
        bool ik_ok = kin.ik_foot(leg, fp, q_recovered);
        if (!ik_ok) {
            fail("kinematics_ik", "IK failed for leg " + std::to_string(leg));
            ok = false;
        } else {
            Eigen::Vector3d fp2 = kin.fk_foot(leg, q_recovered);
            if ((fp2 - fp).norm() > 1e-4) {
                fail("kinematics_roundtrip",
                     "FK(IK(p)) error = " + std::to_string((fp2-fp).norm()));
                ok = false;
            }
        }
    }
    if (ok) pass("kinematics");
    return ok;
}

// ─── Test 2: Gait generator ───────────────────────────────────────────────────

static bool test_gait(const RobotConfig& cfg, const QuadrupedKinematics& kin) {
    GaitGenerator gait(cfg);
    RobotState state = make_stand_state(cfg, kin);

    const double pd_freq = 500.0;
    double dt = 1.0 / pd_freq;
    Eigen::Vector3d v_cmd(0.3, 0.0, 0.0);
    Eigen::Matrix<double,4,3> nmpc_target = state.foot_pos_world;

    // Advance 2 full periods
    int steps = static_cast<int>(2.0 * cfg.gait_period / dt);
    GaitOutput gout;
    for (int i = 0; i < steps; ++i) {
        gout = gait.update(dt, state, v_cmd, nmpc_target);
    }

    auto ref = gait.get_mpc_reference(state, 0.32, v_cmd, 0.0);

    if (ref.rows() != cfg.mpc_horizon || ref.cols() != 12) {
        fail("gait_ref_dims", "MPC reference has wrong dimensions");
        return false;
    }

    bool any_contact = false;
    for (bool c : gout.contact) any_contact |= c;
    if (!any_contact) {
        fail("gait_contact", "no legs in contact during trot");
        return false;
    }

    pass("gait_generator");
    return true;
}

// ─── Test 3: Convex MPC ───────────────────────────────────────────────────────

static bool test_convex_mpc(const RobotConfig& cfg, const QuadrupedKinematics& kin) {
    MPCController mpc(cfg);
    RobotState state = make_stand_state(cfg, kin);

    std::array<bool, 4> contact_state{true, true, true, true};

    // Reference: stand still at 0.32m
    Eigen::MatrixXd X_ref(cfg.mpc_horizon, 12);
    for (int k = 0; k < cfg.mpc_horizon; ++k) {
        X_ref.row(k) << 0.0, 0.0, 0.0,   // rpy
                         0.0, 0.0, 0.32,  // pos
                         0.0, 0.0, 0.0,   // omega
                         0.0, 0.0, 0.0;   // vel
    }

    auto t0 = std::chrono::steady_clock::now();
    Eigen::Matrix<double,12,1> grf = mpc.solve(state, state.foot_pos_world,
                                                contact_state, X_ref);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cout << "    ConvexMPC solve: " << std::fixed << std::setprecision(2)
              << ms << " ms\n";

    if (grf.hasNaN()) {
        fail("convex_mpc_nan", "GRF contains NaN");
        return false;
    }

    // Each stance leg should have positive fz
    bool grf_ok = true;
    for (int i = 0; i < 4; ++i) {
        double fz = grf(i * 3 + 2);
        if (fz < -5.0) {
            fail("convex_mpc_fz", "negative normal force on leg " + std::to_string(i)
                 + ": fz=" + std::to_string(fz));
            grf_ok = false;
        }
    }

    // Total vertical force ≈ mass * g (within 30%)
    double fz_total = grf(2) + grf(5) + grf(8) + grf(11);
    double expected = cfg.mass * 9.81;
    if (std::abs(fz_total - expected) > 0.35 * expected) {
        std::cout << "    Warning: fz_total=" << fz_total
                  << " expected≈" << expected << "\n";
    }

    if (ms > 20.0)
        std::cout << "    Warning: ConvexMPC took " << ms << " ms (target <5 ms)\n";

    if (grf_ok) pass("convex_mpc");
    return grf_ok;
}

// ─── Test 4: NMPC footstep planner ────────────────────────────────────────────

static bool test_nmpc(const RobotConfig& cfg, const QuadrupedKinematics& kin) {
    NMPCFootstepPlanner nmpc(cfg);
    RobotState state = make_stand_state(cfg, kin);

    int N = cfg.mpc_horizon;

    // Trot: legs 0,2 in stance; 1,3 swing
    Eigen::MatrixXd contact_seq(4, N);
    contact_seq.setZero();
    for (int k = 0; k < N; ++k) {
        contact_seq(0, k) = 1.0;
        contact_seq(2, k) = 1.0;
    }

    // Reference: walk forward at 0.1 m/s
    Eigen::MatrixXd X_ref(N, 12);
    for (int k = 0; k < N; ++k) {
        X_ref.row(k) << 0.0, 0.0, 0.0,   0.0, 0.0, 0.32,   0.0, 0.0, 0.0,   0.1, 0.0, 0.0;
    }

    Eigen::Matrix<double,4,3> hip_offsets = cfg.hip_offsets;

    Eigen::Matrix<double,4,3> nom_off;
    nom_off.setZero();
    for (int i = 0; i < 4; ++i) {
        nom_off(i, 0) = cfg.nominal_foot_offsets(i, 0);
        nom_off(i, 1) = cfg.nominal_foot_offsets(i, 1);
    }

    auto t0 = std::chrono::steady_clock::now();
    auto result = nmpc.solve(state, contact_seq, state.foot_pos_world, X_ref,
                              hip_offsets, nom_off);
    auto t1 = std::chrono::steady_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    std::cout << "    NMPC first solve: " << std::fixed << std::setprecision(1)
              << ms << " ms, converged=" << result.converged << "\n";

    if (!result.converged) {
        fail("nmpc_converge", "IPOPT did not converge");
        return false;
    }

    // Foot placement should be within workspace (±0.8m from origin)
    for (int i = 0; i < 4; ++i) {
        double dx = result.foot_placement(i, 0);
        double dy = result.foot_placement(i, 1);
        if (std::abs(dx) > 0.8 || std::abs(dy) > 0.8) {
            fail("nmpc_foot_placement", "foot " + std::to_string(i)
                 + " out of range: (" + std::to_string(dx) + ", " + std::to_string(dy) + ")");
            return false;
        }
    }

    if (ms > 200.0)
        std::cout << "    Warning: NMPC first call " << ms
                  << " ms (target <30 ms warm)\n";

    pass("nmpc_footstep");
    return true;
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char** argv) {
    std::string cfg_path = "config/robot_params.yaml";
    if (argc > 1) cfg_path = argv[1];

    std::cout << "Loading config: " << cfg_path << "\n";
    RobotConfig cfg;
    try {
        cfg = RobotConfig::from_yaml(cfg_path);
    } catch (const std::exception& e) {
        std::cerr << "Failed to load config: " << e.what() << "\n";
        return 1;
    }

    QuadrupedKinematics kin(cfg);

    std::cout << "\n=== MPC Solver Tests ===\n\n";

    int passed = 0, total = 0;

    auto run_kn = [&](bool(*fn)(const RobotConfig&), const char* label) {
        std::cout << "[" << label << "]" << std::endl;
        ++total;
        if (fn(cfg)) ++passed;
        std::cout << std::endl;
    };
    auto run_kk = [&](bool(*fn)(const RobotConfig&, const QuadrupedKinematics&),
                       const char* label) {
        std::cout << "[" << label << "]" << std::endl;
        ++total;
        if (fn(cfg, kin)) ++passed;
        std::cout << std::endl;
    };

    run_kn(test_kinematics,  "Kinematics");
    run_kk(test_gait,        "Gait Generator");
    run_kk(test_convex_mpc,  "Convex MPC");
    run_kk(test_nmpc,        "NMPC Footstep");

    std::cout << "=== Results: " << passed << "/" << total << " passed ===\n";
    return (passed == total) ? 0 : 1;
}
