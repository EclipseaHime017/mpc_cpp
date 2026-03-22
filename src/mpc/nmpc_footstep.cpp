#include "mpc/nmpc_footstep.hpp"
#include <iostream>
#include <cmath>
#include <chrono>

using namespace casadi;

NMPCFootstepPlanner::NMPCFootstepPlanner(const RobotConfig& cfg) : cfg_(cfg) {
    build_nlp();
    // Initialize warm-start to zero
    int N = cfg_.mpc_horizon;
    warm_U_ = DM::zeros(12, N);
    warm_P_ = DM::zeros(4, 2);
}

// Helper: reshape Eigen matrix to casadi DM
static DM eigen_to_dm(const Eigen::MatrixXd& M) {
    DM d(M.rows(), M.cols());
    for (int r = 0; r < (int)M.rows(); ++r)
        for (int c = 0; c < (int)M.cols(); ++c)
            d(r, c) = M(r, c);
    return d;
}

static DM eigen_to_dm(const Eigen::Matrix<double,4,3>& M) {
    DM d(4, 3);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 3; ++c)
            d(r, c) = M(r, c);
    return d;
}

void NMPCFootstepPlanner::build_nlp() {
    int N = cfg_.mpc_horizon;
    double dt = 1.0 / cfg_.mpc_freq;
    double mu = cfg_.mu;

    // ===== Decision variables =====
    // X: [12 x (N+1)]  state trajectory
    MX X      = MX::sym("X",      12, N + 1);
    // U: [12 x N]  GRF trajectory
    MX U      = MX::sym("U",      12, N);
    // P_foot: [4 x 2]  optimized foot placement (world x,y)
    MX P_foot = MX::sym("P_foot",  4, 2);

    // ===== Parameters (set each solve call) =====
    MX X0              = MX::sym("X0",       12);
    MX X_ref_p         = MX::sym("X_ref",    12, N);    // [12 x N]
    MX Contact_Seq     = MX::sym("C_seq",     4, N);
    MX Foot_Pos_Curr   = MX::sym("Fp_curr",   4, 3);
    MX Hip_Off         = MX::sym("Hip_off",   4, 3);
    MX Nom_Off         = MX::sym("Nom_off",   4, 3);

    // ===== Cost =====
    DM Q_mat = DM(DM::diag(
        std::vector<double>(cfg_.weights_Q.data(), cfg_.weights_Q.data() + 12)));
    DM R_mat     = DM::eye(12) * cfg_.weights_R;
    DM R_delta   = DM::eye(12) * (cfg_.weights_R * 100.0);
    double W_foot = 100.0;

    MX cost = 0;
    for (int k = 0; k < N; ++k) {
        MX err = X(Slice(), k) - X_ref_p(Slice(), k);
        cost += mtimes(err.T(), mtimes(Q_mat, err));
        cost += mtimes(U(Slice(), k).T(), mtimes(R_mat, U(Slice(), k)));
        if (k > 0) {
            MX du = U(Slice(), k) - U(Slice(), k - 1);
            cost += mtimes(du.T(), mtimes(R_delta, du));
        }
    }
    // Footstep placement cost (attract to nominal stance)
    int mid = N / 2;
    for (int i = 0; i < 4; ++i) {
        MX yaw_pred  = X(2, mid);
        MX cos_y = cos(yaw_pred), sin_y = sin(yaw_pred);
        MX nom_x = X(3, mid) + Nom_Off(i, 0) * cos_y - Nom_Off(i, 1) * sin_y;
        MX nom_y = X(4, mid) + Nom_Off(i, 0) * sin_y + Nom_Off(i, 1) * cos_y;
        cost += W_foot * (pow(P_foot(i, 0) - nom_x, 2) + pow(P_foot(i, 1) - nom_y, 2));
    }

    // ===== Constraints =====
    std::vector<MX> g_list;
    std::vector<double> g_lb_vec, g_ub_vec;

    auto add_eq = [&](MX expr) {
        g_list.push_back(expr);
        int sz = expr.numel();
        for (int i = 0; i < sz; ++i) { g_lb_vec.push_back(0.0); g_ub_vec.push_back(0.0); }
    };
    auto add_ineq = [&](MX expr, double lb, double ub) {
        g_list.push_back(expr);
        g_lb_vec.push_back(lb);
        g_ub_vec.push_back(ub);
    };

    // Initial state constraint
    add_eq(X(Slice(), 0) - X0);

    const double INF = 1e9;
    DM I_body_dm(3, 3);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            I_body_dm(r, c) = cfg_.I_body(r, c);
    DM I_inv_dm = DM::inv(I_body_dm);
    MX I_inv    = MX(I_inv_dm);

    for (int k = 0; k < N; ++k) {
        MX r = X(0, k), p = X(1, k), y = X(2, k);
        MX pos   = X(Slice(3, 6), k);
        MX omega = X(Slice(6, 9), k);
        MX vel   = X(Slice(9, 12), k);

        // Rotation matrix (ZYX: R = Rz * Ry * Rx)
        MX cy = cos(y), sy = sin(y);
        MX cp = cos(p), sp = sin(p);
        MX cr = cos(r), sr = sin(r);
        MX R_z = MX::vertcat({MX::horzcat({cy, -sy, MX(0)}),
                               MX::horzcat({sy,  cy, MX(0)}),
                               MX::horzcat({MX(0), MX(0), MX(1)})});
        MX R_y = MX::vertcat({MX::horzcat({cp, MX(0), sp}),
                               MX::horzcat({MX(0), MX(1), MX(0)}),
                               MX::horzcat({-sp, MX(0), cp})});
        MX R_x = MX::vertcat({MX::horzcat({MX(1), MX(0), MX(0)}),
                               MX::horzcat({MX(0), cr, -sr}),
                               MX::horzcat({MX(0), sr,  cr})});
        MX R_mat = mtimes(R_z, mtimes(R_y, R_x));

        MX force_sum  = MX::zeros(3);
        MX torque_sum = MX::zeros(3);

        for (int i = 0; i < 4; ++i) {
            MX f_i = U(Slice(i*3, i*3+3), k);
            MX c_flag = Contact_Seq(i, k);

            // Friction cone constraints
            add_ineq(f_i(0) - mu * f_i(2),  -INF, 0.0);
            add_ineq(f_i(0) + mu * f_i(2),  0.0,  INF);
            add_ineq(f_i(1) - mu * f_i(2),  -INF, 0.0);
            add_ineq(f_i(1) + mu * f_i(2),  0.0,  INF);
            // Normal force: 0 <= fz <= fmax * contact_flag
            add_ineq(f_i(2),                0.0, cfg_.f_max);
            add_ineq(f_i(2) - c_flag * cfg_.f_max, -INF, 0.0);

            // Foot position for torque calculation
            MX px_foot, py_foot, pz_foot;
            if (k == 0) {
                // Use actual current foot position at k=0
                px_foot = Foot_Pos_Curr(i, 0);
                py_foot = Foot_Pos_Curr(i, 1);
                pz_foot = Foot_Pos_Curr(i, 2);
            } else {
                // Optimized foot (x,y); z fixed to ground
                px_foot = P_foot(i, 0);
                py_foot = P_foot(i, 1);
                pz_foot = MX(0.0);
            }
            MX r_vec = MX::vertcat({px_foot, py_foot, pz_foot}) - pos;
            force_sum  += f_i;
            torque_sum += MX::cross(r_vec, f_i);
        }

        // Dynamics step (Euler integration)
        MX next_pos   = pos + vel * dt;
        MX next_vel   = vel + (force_sum / cfg_.mass - MX::vertcat({MX(0), MX(0), MX(9.81)})) * dt;
        MX next_rpy   = MX::vertcat({r, p, y}) + mtimes(R_mat.T(), omega) * dt;
        MX next_omega = omega + mtimes(R_mat, mtimes(I_inv, mtimes(R_mat.T(), torque_sum))) * dt;
        MX X_next     = MX::vertcat({next_rpy, next_pos, next_omega, next_vel});

        add_eq(X(Slice(), k + 1) - X_next);
    }

    // Kinematic reachability constraints (foot within ±20cm of hip projection)
    for (int i = 0; i < 4; ++i) {
        MX y_end = X(2, N);
        MX cy = cos(y_end), sy = sin(y_end);
        MX hip_x = X(3, N) + Hip_Off(i, 0) * cy - Hip_Off(i, 1) * sy;
        MX hip_y = X(4, N) + Hip_Off(i, 0) * sy + Hip_Off(i, 1) * cy;
        add_ineq(P_foot(i, 0) - hip_x, -0.20, 0.20);
        add_ineq(P_foot(i, 1) - hip_y, -0.20, 0.20);
    }

    // ===== Assemble NLP =====
    // Decision vector: [X(:), U(:), P_foot(:)]
    MX x_all = MX::vertcat({MX::reshape(X,      -1, 1),
                              MX::reshape(U,      -1, 1),
                              MX::reshape(P_foot, -1, 1)});
    // Parameter vector
    MX p_all = MX::vertcat({X0,
                              MX::reshape(X_ref_p,       -1, 1),
                              MX::reshape(Contact_Seq,   -1, 1),
                              MX::reshape(Foot_Pos_Curr, -1, 1),
                              MX::reshape(Hip_Off,       -1, 1),
                              MX::reshape(Nom_Off,       -1, 1)});
    MX g_all = MX::vertcat(g_list);

    n_vars_ = x_all.numel();
    n_cons_ = (int)g_lb_vec.size();

    g_lb_ = DM(g_lb_vec);
    g_ub_ = DM(g_ub_vec);

    MXDict nlp = {{"x", x_all}, {"f", cost}, {"g", g_all}, {"p", p_all}};

    Dict ipopt_opts;
    ipopt_opts["max_iter"]      = cfg_.ipopt_max_iter;
    ipopt_opts["print_level"]   = 0;
    ipopt_opts["sb"]            = "yes";
    ipopt_opts["tol"]           = cfg_.ipopt_tol;
    ipopt_opts["acceptable_tol"]= cfg_.ipopt_tol * 10.0;

    Dict solver_opts;
    solver_opts["expand"]     = true;
    solver_opts["print_time"] = 0;
    solver_opts["ipopt"]      = ipopt_opts;

    nlp_solver_ = nlpsol("nmpc_solver", "ipopt", nlp, solver_opts);
}

NMPCFootstepPlanner::Result NMPCFootstepPlanner::solve(
    const RobotState& state,
    const Eigen::MatrixXd& contact_seq,
    const Eigen::Matrix<double,4,3>& foot_pos_curr,
    const Eigen::MatrixXd& X_ref,
    const Eigen::Matrix<double,4,3>& hip_offsets,
    const Eigen::Matrix<double,4,3>& nominal_foot_offsets)
{
    auto t0 = std::chrono::steady_clock::now();
    int N = cfg_.mpc_horizon;

    Result result;
    result.f_first_step.setZero();
    result.foot_placement.setZero();

    // Build x0 vector
    std::vector<double> x0_vec = {
        state.rpy[0], state.rpy[1], state.rpy[2],
        state.pos[0], state.pos[1], state.pos[2],
        state.omega[0], state.omega[1], state.omega[2],
        state.vel[0], state.vel[1], state.vel[2]
    };

    // Build parameter vector
    DM X0_dm(x0_vec);

    // X_ref: Eigen [N x 12] -> CasADi [12 x N] (column-major)
    DM X_ref_dm(12, N);
    for (int k = 0; k < N; ++k)
        for (int j = 0; j < 12; ++j)
            X_ref_dm(j, k) = X_ref(k, j);

    DM contact_seq_dm = eigen_to_dm(contact_seq);
    DM foot_pos_dm    = eigen_to_dm(foot_pos_curr);
    DM hip_off_dm     = eigen_to_dm(hip_offsets);
    DM nom_off_dm     = eigen_to_dm(nominal_foot_offsets);

    DM p_val = DM::vertcat({
        DM::reshape(X0_dm,          -1, 1),
        DM::reshape(X_ref_dm,       -1, 1),
        DM::reshape(contact_seq_dm, -1, 1),
        DM::reshape(foot_pos_dm,    -1, 1),
        DM::reshape(hip_off_dm,     -1, 1),
        DM::reshape(nom_off_dm,     -1, 1)
    });

    // Build warm-start initial guess
    DM x0_warm = DM::vertcat({
        DM::zeros(12 * (N + 1)),  // state trajectory (zeros)
        DM::reshape(warm_U_, -1, 1),
        DM::reshape(warm_P_, -1, 1)
    });

    DMDict arg = {
        {"x0",  x0_warm},
        {"p",   p_val},
        {"lbg", g_lb_},
        {"ubg", g_ub_}
    };

    DMDict sol;
    try {
        sol = nlp_solver_(arg);
        result.converged = true;
    } catch (const std::exception& e) {
        // IPOPT didn't converge: use debug (infeasible point) values
        // CasADi returns the last iterate even on failure
        std::cerr << "[NMPC] IPOPT failed: " << e.what() << "\n";
        result.converged = false;
        auto t1 = std::chrono::steady_clock::now();
        result.solve_time_ms = std::chrono::duration<double, std::milli>(t1-t0).count();
        last_solve_ms_ = result.solve_time_ms;
        return result;
    }

    DM x_sol = sol.at("x");

    // Extract U (starts at offset 12*(N+1) in x_sol)
    int u_offset = 12 * (N + 1);
    DM U_sol = DM::reshape(x_sol(Slice(u_offset, u_offset + 12*N)), 12, N);

    // Extract P_foot
    int p_offset = u_offset + 12 * N;
    DM P_sol = DM::reshape(x_sol(Slice(p_offset, p_offset + 8)), 4, 2);

    // First step GRF
    DM f0 = U_sol(Slice(), 0);
    for (int i = 0; i < 12; ++i)
        result.f_first_step[i] = static_cast<double>(f0(i));

    // Foot placement
    for (int i = 0; i < 4; ++i) {
        result.foot_placement(i, 0) = static_cast<double>(P_sol(i, 0));
        result.foot_placement(i, 1) = static_cast<double>(P_sol(i, 1));
    }

    // Update warm-start
    warm_U_ = U_sol;
    warm_P_ = P_sol;

    auto t1 = std::chrono::steady_clock::now();
    result.solve_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    last_solve_ms_ = result.solve_time_ms;
    return result;
}
