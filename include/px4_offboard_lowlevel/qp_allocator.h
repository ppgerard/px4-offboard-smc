/****************************************************************************
 * Constrained control allocation for the tilt-rotor tricopter.
 *
 * WHY THIS EXISTS. The shipped allocator is a 3x3 pseudo-inverse over
 * [tau_x, tau_y, thrust] with the tilt solved separately from tau_z. Three
 * things follow from that shape, and all three are measured problems:
 *
 *   1. There is NO F_x ROW. Tilting a nacelle makes a body-x force, and in the
 *      3x3 form that force is a SIDE EFFECT nothing commands or tracks. So the
 *      one degree of freedom that could hold a head/tail wind without leaning
 *      is unusable: `tilt_2 = -tilt_1` was hardcoded, making common mode
 *      identically zero. Measured 11 Sep on 78 fair draws: STSMC 1/6 against
 *      PX4's 6/6 at 7.5 m/s NOSE-ON, where the disturbance is only ~2.4 N.
 *
 *   2. LIMITS ARE APPLIED AFTER THE SOLVE. omega_sq that comes out negative is
 *      clamped to zero and the tilt is clamped to +-6.5 deg, so the delivered
 *      wrench can point somewhere quite different from the demand, with no
 *      relation to what was asked. Here the limits are CONSTRAINTS, so the
 *      solution is feasible by construction and is the closest reachable wrench
 *      in the weighted sense.
 *
 *   3. SATURATION IS ONE FLAG. `allocation_saturated_` freezes BOTH reaching
 *      integrals whenever any clamp fires -- including when only the tilt (the
 *      YAW axis) saturated, which says nothing about whether the translational
 *      wrench arrived. This reports saturation PER AXIS instead.
 *
 * WHAT IT DOES NOT DO. It is still model-based: it uses the same ct/km/lever
 * arms, so a wrong ct or km produces the same allocation error as before. What
 * changes is how the error DEGRADES, not whether it exists. Parameter error
 * remains the feedback loops' job.
 *
 * ACTUATOR LAG enters as RATE CONSTRAINTS rather than as dynamics: the reachable
 * change in one cycle is bounded by the rotor time constant and the servo's
 * 90 deg/s limit. Today both are imposed by clamping AFTER the solve, i.e. the
 * commanded wrench is one the actuator provably cannot deliver and the
 * difference is lost silently.
 ****************************************************************************/
#ifndef PX4_OFFBOARD_QP_ALLOCATOR_H
#define PX4_OFFBOARD_QP_ALLOCATOR_H

#include <algorithm>
#include <cmath>
#include <Eigen/Dense>

namespace px4_offboard {

// Geometry and constants the wrench model needs. All lengths in metres, in the
// body FLU frame, with the rotor positions taken at ZERO tilt -- the tilt
// dependence is applied inside the model.
struct TricopterGeometry {
    double ct = 1.6e-5;          // thrust per omega^2 [N.s^2/rad^2]
    double km = 0.018;           // drag-torque / thrust ratio [m]
    // rotor positions at zero tilt: front-right, front-left, tail
    Eigen::Vector3d r0{ 0.20, -0.215, 0.0};
    Eigen::Vector3d r1{ 0.20,  0.215, 0.0};
    Eigen::Vector3d r2{-0.40,  0.0,   0.0};
    // The front rotors hang this far BELOW the tilt axis they swing about, so
    // tilting moves the thrust application point. model.sdf: motor_N at
    // z = 0.045, rotor_N at z = 0.0.
    double tilt_axis_to_rotor_z = 0.045;
    // Polar inertia of one rotor+prop about its spin axis [kg.m^2], and the
    // spin rate used for the gyroscopic term. Tilting a SPINNING rotor about
    // body y precesses about body x: M = J*omega_spin x omega_tilt. At 698 rad/s
    // that is 0.37 N.m for the pair at 90 deg/s and 2.45 N.m at 600 -- against a
    // hover trim torque of 0.03 N.m. It is a known, computable term, so it
    // belongs in the model as feedforward rather than being left for the roll
    // loop to discover as a disturbance. 0 disables it.
    double rotor_izz = 0.000167604;
    double rotor_spin_rate = 0.0;      // [rad/s]; 0 disables the gyroscopic term
    // Spin direction of each rotor's drag torque about body +z, MEASURED on the
    // aircraft: front-right CCW, front-left CW, tail CW. The simulator's tail
    // turns the other way -- see CLAUDE.md.
    Eigen::Vector3d spin{-1.0, 1.0, 1.0};
};

// The five actuators: three rotor speeds SQUARED (thrust is linear in these)
// and the two tilt angles. Squared speed rather than speed because every
// effectiveness term is linear in omega^2, which keeps the model affine in four
// of the five variables.
struct AllocState {
    Eigen::Vector3d w{0.0, 0.0, 0.0};   // omega^2 [rad^2/s^2]
    double t0 = 0.0, t1 = 0.0;          // front tilt angles [rad]
};

// Box and rate limits. Rate limits are what carry the actuator lag.
struct AllocLimits {
    double w_min = 0.0, w_max = 1.3e6;     // omega^2
    double tilt_min = -0.1134, tilt_max = 1.5708;   // -6.5 .. 90 deg
    double dw_max = 1e9;                   // max |change| in omega^2 per cycle
    double dtilt_max = 1e9;                // max |change| in tilt per cycle [rad]
};

// Exact wrench produced by a given actuator state: [tau_x, tau_y, tau_z, F_x, F_z].
// Derivation, for rotor i at r_i with tilt t_i and spin sign s_i:
//     d_i = (sin t, 0, cos t)                 thrust direction
//     F_i = ct*w_i*d_i
//     Q_i = s_i*km*ct*w_i*d_i                 drag torque, along the same axis
//     M_i = r_i x F_i + Q_i
// r_i is tilt-dependent for the front two (see tilt_axis_to_rotor_z).
inline Eigen::Matrix<double,5,1> allocWrench(const TricopterGeometry &g,
                                             const AllocState &x,
                                             const AllocState *x_prev = nullptr,
                                             double dt = 0.0) {
    Eigen::Matrix<double,5,1> y = Eigen::Matrix<double,5,1>::Zero();
    const double t[3] = {x.t0, x.t1, 0.0};
    const Eigen::Vector3d r_nom[3] = {g.r0, g.r1, g.r2};
    for (int i = 0; i < 3; ++i) {
        const double c = std::cos(t[i]), s = std::sin(t[i]);
        Eigen::Vector3d r = r_nom[i];
        if (i < 2) {                       // only the front two tilt
            r.x() -= g.tilt_axis_to_rotor_z * s;
            r.z() += g.tilt_axis_to_rotor_z * (1.0 - c);
        }
        const double T = g.ct * x.w[i];
        const Eigen::Vector3d d(s, 0.0, c);
        const Eigen::Vector3d F = T * d;
        const Eigen::Vector3d M = r.cross(F) + g.spin[i] * g.km * T * d;
        y(0) += M.x();  y(1) += M.y();  y(2) += M.z();
        y(3) += F.x();  y(4) += F.z();
    }
    // GYROSCOPIC precession from tilting the spinning rotors. The spin axis is
    // body z rotated by the tilt, the tilt rate is about body y, so the moment
    // is along their cross product -- predominantly body x (roll). The two front
    // rotors spin OPPOSITE ways here, so a COMMON-MODE tilt largely cancels and
    // a DIFFERENTIAL tilt does not: exactly the opposite of the force coupling,
    // and a good reason to have it in the model rather than in the roll loop.
    if (x_prev != nullptr && dt > 0.0 && g.rotor_spin_rate > 0.0 && g.rotor_izz > 0.0) {
        const double rate[2] = {(x.t0 - x_prev->t0) / dt, (x.t1 - x_prev->t1) / dt};
        const double tl[2] = {x.t0, x.t1};
        for (int i = 0; i < 2; ++i) {
            const Eigen::Vector3d spin_axis(std::sin(tl[i]), 0.0, std::cos(tl[i]));
            const Eigen::Vector3d w_spin = g.spin[i] * g.rotor_spin_rate * spin_axis;
            const Eigen::Vector3d w_tilt(0.0, rate[i], 0.0);
            const Eigen::Vector3d M = g.rotor_izz * w_spin.cross(w_tilt);
            y(0) += M.x();  y(1) += M.y();  y(2) += M.z();
        }
    }
    return y;
}

// Jacobian d(wrench)/d(actuators), by central differences on the exact model.
// Analytic derivatives are available but the numerical form cannot drift out of
// agreement with allocWrench(), and at 5 columns it costs ~10 model evaluations
// per cycle -- nothing at 100 Hz.
inline Eigen::Matrix<double,5,5> allocJacobian(const TricopterGeometry &g,
                                               const AllocState &x,
                                               const AllocState *x_prev = nullptr,
                                               double dt = 0.0) {
    Eigen::Matrix<double,5,5> J;
    const double hw = 1.0, ht = 1e-5;
    for (int k = 0; k < 5; ++k) {
        AllocState a = x, b = x;
        double h;
        if (k < 3) { a.w[k] -= hw; b.w[k] += hw; h = hw; }
        else if (k == 3) { a.t0 -= ht; b.t0 += ht; h = ht; }
        else { a.t1 -= ht; b.t1 += ht; h = ht; }
        J.col(k) = (allocWrench(g, b, x_prev, dt) - allocWrench(g, a, x_prev, dt)) / (2.0 * h);
    }
    return J;
}


// What the solve produced, and what it had to give up.
struct AllocResult {
    AllocState x;                              // the commanded actuator state
    Eigen::Matrix<double,5,1> achieved;        // wrench actually produced by x
    Eigen::Matrix<double,5,1> residual;        // desired - achieved, per axis
    bool at_limit[5] = {false,false,false,false,false};  // per ACTUATOR
    int iterations = 0;
};

// Weights on the five wrench rows [tau_x, tau_y, tau_z, F_x, F_z]. Yaw is the
// axis to give up first on this airframe: it is served by differential tilt,
// which is also the actuator that makes body-x force, so under saturation the
// two compete directly. A low tau_z weight spends the tilt on position rather
// than heading, which is the right trade for a LANDING -- the heading only has
// to be roughly right, the position has to be centimetres.
struct AllocWeights {
    // [tau_x, tau_y, tau_z, F_x, F_z]. These are NOT all equally valuable and the
    // first version wrongly said they were.
    //
    //   F_z = 10.0  Losing body-z force loses the AIRCRAFT. Near-inviolable.
    //               A hard equality would be the textbook form, but it makes the
    //               problem INFEASIBLE whenever F_z alone is unreachable (gust
    //               recovery, low battery) and forces a fallback path -- and
    //               untested fallback paths are where silent misbehaviour lives.
    //               A heavy weight gives the same priority, stays always
    //               feasible, and degrades continuously.
    //   F_x =  0.5  Opportunistic. Losing it costs POSITION, which the attitude
    //               then picks up as lean: the shortfall degrades to exactly the
    //               behaviour the 3x3 path already has, so the tilt can only add.
    //   tau_z= 0.25 Cheapest. Yaw is served by differential tilt -- the same
    //               actuator that makes body-x -- so under saturation the two
    //               compete directly, and a landing needs the heading roughly
    //               right and the position to centimetres.
    //
    // Measured 11 Sep with all five at ~1.0: the allocator traded F_z away for
    // F_x every cycle, and the QP at the SAME tilt authority as the 3x3 path
    // landed 0/3 where the 3x3 landed 6/6 -- with |s_xy| p50 of 1.0099/1.0172/
    // 1.0115, i.e. deterministic, which is what a wrong objective looks like
    // against a rig whose honest scatter at that cell is 35x.
    Eigen::Matrix<double,5,1> w_out =
        (Eigen::Matrix<double,5,1>() << 1.0, 1.0, 0.25, 0.5, 10.0).finished();
    double lambda = 1e-9;   // Tikhonov on the step; keeps the solve conditioned
};

// Box-constrained least squares, min ||A dx - r||^2 + lambda||dx||^2 s.t. lo<=dx<=hi.
// Active-set: solve on the free variables, clamp, release any variable whose
// gradient points back INTO the box, repeat. Five variables converges in a
// handful of passes; the iteration cap makes it bounded-time for a control loop.
inline Eigen::Matrix<double,5,1> solveBoxLS(const Eigen::Matrix<double,5,5> &A,
                                            const Eigen::Matrix<double,5,1> &r,
                                            const Eigen::Matrix<double,5,1> &lo,
                                            const Eigen::Matrix<double,5,1> &hi,
                                            double lambda) {
    Eigen::Matrix<double,5,1> dx = Eigen::Matrix<double,5,1>::Zero();
    bool fixed[5] = {false,false,false,false,false};
    for (int pass = 0; pass < 8; ++pass) {
        // Solve on the free set only.
        Eigen::Matrix<double,5,5> H = A.transpose() * A;
        Eigen::Matrix<double,5,1> g = A.transpose() * r;
        for (int i = 0; i < 5; ++i) { H(i,i) += lambda; }
        for (int i = 0; i < 5; ++i) {
            if (!fixed[i]) { continue; }
            // pin variable i at its current dx(i): zero its row/col, keep value
            for (int j = 0; j < 5; ++j) { H(i,j) = 0.0; H(j,i) = 0.0; }
            H(i,i) = 1.0; g(i) = dx(i);
            for (int j = 0; j < 5; ++j) {
                if (j != i) { g(j) -= A.col(i).dot(A.col(j)) * dx(i); }
            }
        }
        Eigen::Matrix<double,5,1> cand = H.ldlt().solve(g);
        if (!cand.allFinite()) { break; }
        // Clamp and decide the new active set.
        bool changed = false;
        for (int i = 0; i < 5; ++i) {
            double v = std::clamp(cand(i), lo(i), hi(i));
            if (v != cand(i) && !fixed[i]) { fixed[i] = true; changed = true; }
            cand(i) = v;
        }
        dx = cand;
        if (!changed) { break; }
    }
    return dx;
}

// One allocation step. Gauss-Newton on the exact wrench model, warm-started
// from the previous actuator state, with the box built from BOTH the absolute
// limits and the per-cycle rate limits -- the latter is where actuator lag
// enters, so the result is a wrench the actuators can actually reach this cycle.
inline AllocResult allocate(const TricopterGeometry &g,
                            const AllocLimits &lim,
                            const AllocWeights &wt,
                            const AllocState &x_prev,
                            const Eigen::Matrix<double,5,1> &desired,
                            int max_iter = 3,
                            double dt = 0.0) {
    AllocResult out;
    AllocState x = x_prev;
    for (int it = 0; it < max_iter; ++it) {
        const Eigen::Matrix<double,5,1> y = allocWrench(g, x, &x_prev, dt);
        const Eigen::Matrix<double,5,5> J = allocJacobian(g, x, &x_prev, dt);
        const Eigen::Matrix<double,5,1> r = wt.w_out.asDiagonal() * (desired - y);
        const Eigen::Matrix<double,5,5> A = wt.w_out.asDiagonal() * J;
        // Box on the STEP: absolute limits and rate limits, intersected.
        Eigen::Matrix<double,5,1> lo, hi;
        const double cur[5] = {x.w[0], x.w[1], x.w[2], x.t0, x.t1};
        const double prv[5] = {x_prev.w[0], x_prev.w[1], x_prev.w[2], x_prev.t0, x_prev.t1};
        for (int i = 0; i < 5; ++i) {
            const bool is_w = (i < 3);
            const double amin = is_w ? lim.w_min : lim.tilt_min;
            const double amax = is_w ? lim.w_max : lim.tilt_max;
            const double rate = is_w ? lim.dw_max : lim.dtilt_max;
            lo(i) = std::max(amin - cur[i], (prv[i] - rate) - cur[i]);
            hi(i) = std::min(amax - cur[i], (prv[i] + rate) - cur[i]);
            if (lo(i) > hi(i)) { lo(i) = hi(i) = 0.0; }
        }
        // SCALE before solving. omega^2 is O(5e5) and tilt is O(0.1 rad) -- six
        // orders of magnitude apart, which makes A'A hopelessly conditioned and
        // leaves Gauss-Newton stalling with a visible residual even on a demand
        // that is exactly reachable. Solving in scaled coordinates fixes it;
        // the scale is derived from the limits so it needs no tuning.
        Eigen::Matrix<double,5,1> sc;
        const double sw = std::max(1.0, lim.w_max * 0.1);
        sc << sw, sw, sw, 1.0, 1.0;
        Eigen::Matrix<double,5,5> As = A * sc.asDiagonal();
        Eigen::Matrix<double,5,1> los = lo.cwiseQuotient(sc), his = hi.cwiseQuotient(sc);
        const Eigen::Matrix<double,5,1> dxs = solveBoxLS(As, r, los, his, wt.lambda);
        const Eigen::Matrix<double,5,1> dx = sc.cwiseProduct(dxs);
        x.w[0] += dx(0); x.w[1] += dx(1); x.w[2] += dx(2);
        x.t0   += dx(3); x.t1   += dx(4);
        out.iterations = it + 1;
        if (dx.norm() < 1e-9) { break; }
    }
    // Report which ACTUATORS ended on a bound -- per actuator, so a saturated
    // tilt (the yaw axis) no longer has to be reported as "the translational
    // wrench did not arrive", which is what one shared flag forced.
    const double fin[5] = {x.w[0], x.w[1], x.w[2], x.t0, x.t1};
    for (int i = 0; i < 5; ++i) {
        const bool is_w = (i < 3);
        const double amin = is_w ? lim.w_min : lim.tilt_min;
        const double amax = is_w ? lim.w_max : lim.tilt_max;
        out.at_limit[i] = (fin[i] <= amin + 1e-9) || (fin[i] >= amax - 1e-9);
    }
    out.x = x;
    out.achieved = allocWrench(g, x, &x_prev, dt);
    out.residual = desired - out.achieved;
    return out;
}

}  // namespace px4_offboard
#endif  // PX4_OFFBOARD_QP_ALLOCATOR_H
