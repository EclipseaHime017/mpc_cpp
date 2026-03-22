#include "mpc/mpc_controller.hpp"
#include "common/math_utils.hpp"

// Use the OSQP v1.0 C API directly to avoid OsqpEigen wrapper ABI issues.
#include <osqp/osqp.h>
#include <Eigen/Sparse>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

static constexpr int NX = 12;  // state dimension
static constexpr int NU = 12;  // control dimension (4 legs x 3 forces)

MPCController::MPCController(const RobotConfig& cfg) : cfg_(cfg) {}

void MPCController::build_system(const RobotState& state,
                                  const Eigen::Matrix<double,4,3>& foot_pos,
                                  Eigen::MatrixXd& Ad,
                                  Eigen::MatrixXd& Bd,
                                  Eigen::VectorXd& gd) const
{
    double dt  = 1.0 / cfg_.mpc_freq;
    double yaw = state.rpy[2];

    // Continuous A_c (12x12): angular kinematics + linear kinematics
    // State: [roll, pitch, yaw, px, py, pz, wx, wy, wz, vx, vy, vz]
    Eigen::MatrixXd A_c = Eigen::MatrixXd::Zero(NX, NX);
    // d(rpy)/dt = R_z^T * omega  (linearized around yaw)
    double cy = std::cos(yaw), sy = std::sin(yaw);
    Eigen::Matrix3d R_z_T;
    R_z_T <<  cy, sy, 0,
             -sy, cy, 0,
               0,  0, 1;
    A_c.block<3,3>(0, 6) = R_z_T;   // d(rpy)/dt = R_z^T * omega
    A_c.block<3,3>(3, 9) = Eigen::Matrix3d::Identity();  // d(pos)/dt = vel

    // Continuous B_c (12x12): angular + linear dynamics
    Eigen::MatrixXd B_c = Eigen::MatrixXd::Zero(NX, NU);
    Eigen::Vector3d pos_com_world = state.pos + state.rot_mat * cfg_.com_offset;
    Eigen::Matrix3d I_world_inv = state.rot_mat * cfg_.I_body.inverse() * state.rot_mat.transpose();

    for (int i = 0; i < 4; ++i) {
        // Fix 1: always build for all 4 legs; contact enforced via bounds
        Eigen::Vector3d r = foot_pos.row(i).transpose() - pos_com_world;
        Eigen::Matrix3d r_mat = math_utils::skew(r);
        int idx = i * 3;
        B_c.block<3,3>(6, idx) = I_world_inv * r_mat;        // torque -> angular acc
        B_c.block<3,3>(9, idx) = Eigen::Matrix3d::Identity() / cfg_.mass;  // force -> linear acc
    }

    // Euler discretization
    Ad = Eigen::MatrixXd::Identity(NX, NX) + A_c * dt;
    Bd = B_c * dt;
    gd = Eigen::VectorXd::Zero(NX);
    gd[11] = -9.81 * dt;  // gravity in vz
}

Eigen::Matrix<double, 12, 1> MPCController::solve(
    const RobotState& state,
    const Eigen::Matrix<double,4,3>& foot_positions,
    const std::array<bool, 4>& contact_state,
    const Eigen::MatrixXd& desired_traj)
{
    auto t0 = std::chrono::steady_clock::now();

    const int N  = cfg_.mpc_horizon;
    const int nx = NX, nu = NU;
    const int total_x = nx * N;
    const int total_u = nu * N;

    // Build discrete system
    Eigen::MatrixXd Ad, Bd;
    Eigen::VectorXd gd;
    build_system(state, foot_positions, Ad, Bd, gd);

    // Initial state x0
    Eigen::VectorXd x0(nx);
    x0.segment<3>(0)  = state.rpy;
    x0.segment<3>(3)  = state.pos;
    x0.segment<3>(6)  = state.omega;
    x0.segment<3>(9)  = state.vel;

    // Build S_u (total_x x total_u) and S_x (total_x x nx)
    Eigen::MatrixXd S_u = Eigen::MatrixXd::Zero(total_x, total_u);
    Eigen::MatrixXd S_x = Eigen::MatrixXd::Zero(total_x, nx);
    Eigen::VectorXd S_g = Eigen::VectorXd::Zero(total_x);

    Eigen::MatrixXd curr_A = Ad;
    Eigen::VectorXd curr_g = gd;

    for (int k = 0; k < N; ++k) {
        int row = k * nx;
        S_x.block(row, 0, nx, nx) = curr_A;
        S_g.segment(row, nx)      = curr_g;

        // S_u[row:row+nx, j*nu:(j+1)*nu] = Ad^(k-j) * Bd
        Eigen::MatrixXd A_pow_B = Bd;
        for (int j = k; j >= 0; --j) {
            S_u.block(row, j * nu, nx, nu) = A_pow_B;
            A_pow_B = Ad * A_pow_B;
        }

        if (k < N - 1) {
            curr_A = Ad * curr_A;
            curr_g = Ad * curr_g + gd;
        }
    }

    // Fix 2: X_ref flattened row-major (no transpose)
    // desired_traj: [N x 12]
    Eigen::VectorXd X_ref(total_x);
    for (int k = 0; k < N; ++k)
        X_ref.segment(k * nx, nx) = desired_traj.row(k).transpose();

    // Cost matrices (diagonal)
    Eigen::VectorXd Q_diag(total_x);
    for (int k = 0; k < N; ++k)
        Q_diag.segment(k * nx, nx) = cfg_.weights_Q;

    // P = S_u^T * Q * S_u + R_bar (dense, small: 120x120 for N=10)
    Eigen::MatrixXd P_dense =
        S_u.transpose() * Q_diag.asDiagonal() * S_u +
        Eigen::MatrixXd::Identity(total_u, total_u) * cfg_.weights_R;

    // q_vec = S_u^T * Q * (S_x*x0 + S_g - X_ref)
    Eigen::VectorXd delta0 = S_x * x0 + S_g - X_ref;
    Eigen::VectorXd q_vec  = S_u.transpose() * (Q_diag.asDiagonal() * delta0);

    // Make P symmetric (numerical symmetrization)
    P_dense = 0.5 * (P_dense + P_dense.transpose());

    // Constraint matrix: [5 constraints/leg x 4 legs x N steps]
    // [fz, fx-mu*fz, fy-mu*fz, fx+mu*fz, fy+mu*fz] <= [fmax, 0, 0, inf, inf]
    const double mu = cfg_.mu;
    const int n_leg_cons = 5;
    const int n_cons = n_leg_cons * 4 * N;

    // OSQP v1.0 C API (float precision: OSQPFloat = float).
    // Using C API directly to avoid OsqpEigen wrapper ABI incompatibility.
    using CF = OSQPFloat;
    static constexpr CF INF_CF = static_cast<CF>(1e9);

    // --- Build constraint matrix A in CSC format ---
    // Triplet accumulation
    std::vector<OSQPInt>   A_row, A_col_ptr_tmp;
    std::vector<CF>        A_val;
    // Build as triplets first, then convert to CSC
    std::vector<std::tuple<int,int,double>> triplets;
    triplets.reserve(n_cons * 3);

    std::vector<CF> l_cf(n_cons), u_cf(n_cons);

    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < 4; ++i) {
            int row   = (k * 4 + i) * n_leg_cons;
            int col_b = k * nu + i * 3;
            triplets.emplace_back(row+0, col_b+2,  1.0);
            triplets.emplace_back(row+1, col_b+0,  1.0);
            triplets.emplace_back(row+1, col_b+2, -mu);
            triplets.emplace_back(row+2, col_b+1,  1.0);
            triplets.emplace_back(row+2, col_b+2, -mu);
            triplets.emplace_back(row+3, col_b+0,  1.0);
            triplets.emplace_back(row+3, col_b+2,  mu);
            triplets.emplace_back(row+4, col_b+1,  1.0);
            triplets.emplace_back(row+4, col_b+2,  mu);

            if (!contact_state[i]) {
                for (int r = 0; r < n_leg_cons; ++r) {
                    l_cf[row+r] = CF(0); u_cf[row+r] = CF(0);
                }
            } else {
                l_cf[row+0] = static_cast<CF>(cfg_.f_min);
                u_cf[row+0] = static_cast<CF>(cfg_.f_max);
                l_cf[row+1] = -INF_CF;  u_cf[row+1] = CF(0);
                l_cf[row+2] = -INF_CF;  u_cf[row+2] = CF(0);
                l_cf[row+3] = CF(0);    u_cf[row+3] = INF_CF;
                l_cf[row+4] = CF(0);    u_cf[row+4] = INF_CF;
            }
        }
    }

    // Convert triplets to Eigen sparse, then extract CSC arrays
    Eigen::SparseMatrix<double> A_sp(n_cons, total_u);
    {
        std::vector<Eigen::Triplet<double>> eig_trips;
        eig_trips.reserve(triplets.size());
        for (auto& [r, c, v] : triplets) eig_trips.emplace_back(r, c, v);
        A_sp.setFromTriplets(eig_trips.begin(), eig_trips.end());
        A_sp.makeCompressed();
    }

    // Hessian: upper triangular part only (OSQP requirement)
    Eigen::MatrixXd P_upper_dense = P_dense.triangularView<Eigen::Upper>();
    Eigen::SparseMatrix<double> P_upper = P_upper_dense.sparseView(1e-12, 1.0);
    P_upper.makeCompressed();

    // Convert to OSQP float arrays
    int P_nnz = (int)P_upper.nonZeros();
    int A_nnz = (int)A_sp.nonZeros();
    std::vector<CF>      P_val(P_nnz), A_val2(A_nnz);
    std::vector<CF>      q_cf(total_u);
    std::vector<OSQPInt> P_row(P_nnz), P_colptr(total_u + 1);
    std::vector<OSQPInt> A_row2(A_nnz), A_colptr(total_u + 1);

    for (int i = 0; i < P_nnz;    ++i) P_val[i]  = static_cast<CF>(P_upper.valuePtr()[i]);
    for (int i = 0; i < P_nnz;    ++i) P_row[i]  = static_cast<OSQPInt>(P_upper.innerIndexPtr()[i]);
    for (int i = 0; i <= total_u; ++i) P_colptr[i] = static_cast<OSQPInt>(P_upper.outerIndexPtr()[i]);
    for (int i = 0; i < A_nnz;    ++i) A_val2[i] = static_cast<CF>(A_sp.valuePtr()[i]);
    for (int i = 0; i < A_nnz;    ++i) A_row2[i] = static_cast<OSQPInt>(A_sp.innerIndexPtr()[i]);
    for (int i = 0; i <= total_u; ++i) A_colptr[i] = static_cast<OSQPInt>(A_sp.outerIndexPtr()[i]);
    for (int i = 0; i < total_u;  ++i) q_cf[i]   = static_cast<CF>(q_vec[i]);

    // Build OSQP CSC structs (stack, no allocation)
    OSQPCscMatrix P_csc{}, A_csc{};
    P_csc.m = total_u;  P_csc.n = total_u;  P_csc.nzmax = P_nnz;
    P_csc.x = P_val.data();  P_csc.i = P_row.data();  P_csc.p = P_colptr.data();
    P_csc.nz = -1;  // CSC format
    A_csc.m = n_cons;   A_csc.n = total_u;  A_csc.nzmax = A_nnz;
    A_csc.x = A_val2.data(); A_csc.i = A_row2.data(); A_csc.p = A_colptr.data();
    A_csc.nz = -1;

    // OSQP settings
    OSQPSettings settings{};
    osqp_set_default_settings(&settings);
    settings.verbose       = 0;
    settings.warm_starting = 0;
    settings.max_iter      = 400;
    settings.eps_abs       = CF(1e-3);
    settings.eps_rel       = CF(1e-3);
    settings.polishing     = 0;

    // Fix 3: re-setup OSQP every call
    OSQPSolver* osqp_solver = nullptr;
    OSQPInt ret = osqp_setup(&osqp_solver, &P_csc, q_cf.data(),
                              &A_csc, l_cf.data(), u_cf.data(),
                              n_cons, total_u, &settings);
    if (ret != 0 || osqp_solver == nullptr) {
        auto t1 = std::chrono::steady_clock::now();
        last_solve_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
        return last_result_;
    }

    osqp_solve(osqp_solver);

    bool solved = (osqp_solver->info->status_val == OSQP_SOLVED ||
                   osqp_solver->info->status_val == OSQP_SOLVED_INACCURATE);

    if (solved && osqp_solver->solution && osqp_solver->solution->x) {
        for (int i = 0; i < 12; ++i)
            last_result_[i] = static_cast<double>(osqp_solver->solution->x[i]);
        if (last_result_.hasNaN()) last_result_.setZero();
    }

    osqp_cleanup(osqp_solver);

    auto t1 = std::chrono::steady_clock::now();
    last_solve_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return last_result_;
}
