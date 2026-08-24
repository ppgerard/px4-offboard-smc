/****************************************************************************
 * Super-twisting reaching law: discretisation and the generalised linear terms.
 *
 * ROS-free and Eigen-only, so it is unit-tested offline (see
 * test/sta_reaching_law_test.cpp) rather than by a four-minute flight. Both
 * control laws call it for both surfaces, so the translational and rotational
 * branches cannot drift apart.
 *
 * Two things are configurable here, and BOTH default to exactly the law that
 * shipped, bit for bit, so each A/Bs against one binary:
 *
 *   1. DISCRETISATION (roadmap item 15). The explicit scheme evaluates sign(s)
 *      at the current sample and integrates w afterwards, so the state that
 *      lands on the surface overshoots it and alternates sign at the sampling
 *      rate. The implicit scheme resolves the switching at the NEXT sample,
 *      which is a scalar equation with a closed-form root here, and lands on
 *      the surface exactly.
 *
 *   2. LINEAR TERMS (roadmap item 10, generalised STA). k3 and k4 add -k3*s to
 *      the proportional branch and -k4*s to the integral branch. A pure STA has
 *      no linear damping at all -- its only gain near the surface is the sqrt
 *      branch's, which is unbounded as s -> 0 -- and that missing damping is
 *      the signature of a distinct-frequency limit cycle rather than
 *      Nyquist-rate dither.
 *
 * The value of item 10 is NOT that it quiets the loop by itself. It ADDS gain
 * at the crossover and on its own should make chattering worse. Its value is
 * that the linear term carries the disturbance rejection at moderate |s|, which
 * is what allows k1 -- the gain that sets the incremental gain NEAR the surface,
 * and therefore the chattering -- to come down.
 ****************************************************************************/

#ifndef PX4_OFFBOARD_STA_REACHING_LAW_H
#define PX4_OFFBOARD_STA_REACHING_LAW_H

#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Eigen>

namespace px4_offboard {

// Per-axis super-twisting gains. k3/k4 are the generalised (linear) terms and
// are zero for the classical law.
struct StaGains {
    Eigen::Vector3d k1 = Eigen::Vector3d::Zero();   // sqrt branch
    Eigen::Vector3d k2 = Eigen::Vector3d::Zero();   // integral branch
    Eigen::Vector3d k3 = Eigen::Vector3d::Zero();   // linear proportional
    Eigen::Vector3d k4 = Eigen::Vector3d::Zero();   // linear integral
};

// Super-twisting integral update w += increment, with the two bounds the plain
// recursion leaves out: |w| limited per axis, and no integration while the
// actuators are saturated (conditional integration).
//
// Freezing outright would trap the state -- wound up, and unable to unwind for
// as long as the saturation lasts -- so an increment that shrinks |w| is still
// applied. Only growth is held back.
inline void integrateStaState(Eigen::Vector3d &w, const Eigen::Vector3d &increment,
                              const Eigen::Vector3d &limit, bool saturated) {
    for (int i = 0; i < 3; ++i) {
        const bool winds_up = increment(i) * w(i) > 0.0;
        if (saturated && winds_up) {
            continue;
        }
        w(i) = std::clamp(w(i) + increment(i), -limit(i), limit(i));
    }
}

// Resolve the implicit switching for one axis.
//
// The plant seen by the surface is  s_dot = b*u + d,  and implicit Euler on the
// super-twisting law gives, with  P = s + dt*b*w  the free prediction and
// sigma = s[k+1]:
//
//     sigma = P - dt*b*k1*sqrt(|sigma|)*sign(sigma) - dt^2*b*k2*xi,   xi in Sign(sigma)
//
// For sigma > 0 this is a quadratic in v = sqrt(sigma) with one non-negative
// root; for sigma < 0 it is its mirror. When |P| <= dt^2*b*k2 the only
// consistent solution is sigma = 0 with xi = P/(dt^2*b*k2): the state lands ON
// the surface and the selection is linear in P.
//
// That linear region is what removes the sampling-rate alternation, and its
// width is dt^2*b*k2 -- set by the step and the gain, NOT chosen. It is not a
// boundary layer: a boundary layer is a tuned width that trades accuracy for
// smoothness, this is the exact solution of the same law at the same gains.
struct ImplicitSwitch {
    double sigma;   // the resolved next-sample surface
    double xi;      // the switching selection, in [-1, 1]
};

inline ImplicitSwitch resolveImplicitSwitch(double s, double w, double k1, double k2,
                                            double dt, double b) {
    const double prediction = s + dt * b * w;
    const double dead = dt * dt * b * k2;
    if (dead <= 0.0) {
        // No integral branch: the sqrt branch alone still has a closed form.
        const double a = dt * b * k1;
        const double magnitude = std::abs(prediction);
        const double root = 0.5 * (-a + std::sqrt(a * a + 4.0 * magnitude));
        const double sigma = (root > 0.0 ? root * root : 0.0);
        const double sign = (prediction > 0.0) - (prediction < 0.0);
        return {sign * sigma, sign};
    }
    if (std::abs(prediction) <= dead) {
        return {0.0, prediction / dead};
    }
    const double sign = (prediction > 0.0) - (prediction < 0.0);
    const double a = dt * b * k1;
    // |sigma| solves  v^2 + a*v - (|P| - dead) = 0  with v = sqrt(|sigma|) >= 0.
    const double residual = std::abs(prediction) - dead;
    const double root = 0.5 * (-a + std::sqrt(a * a + 4.0 * residual));
    const double sigma = (root > 0.0 ? root * root : 0.0);
    return {sign * sigma, sign};
}

// One step of the reaching law. Returns the control u and advances w through
// integrateStaState(), so the bound and the anti-windup apply identically to
// every variant.
//
// `b` is the control effectiveness per axis (ds/dt per unit u): 1/mass for the
// translational surface, 1/inertia for the rotational one. It is only read by
// the implicit scheme -- the explicit one does not need a plant model, which is
// exactly why it was the starting point.
//
// Note the ORDER for the implicit scheme: w is advanced BEFORE u is assembled,
// so u carries w[k+1]. That removes a full sample of delay from the integral
// branch, which at a 100 Hz loop is 10 ms -- worth about 40 degrees of phase at
// the ~11 Hz crossover this airframe oscillates at, and plausibly more valuable
// here than the overshoot band the scheme was designed to remove.
inline Eigen::Vector3d staReachingStep(const Eigen::Vector3d &s,
                                       Eigen::Vector3d &w,
                                       const StaGains &gains,
                                       const Eigen::Vector3d &b,
                                       const Eigen::Vector3d &w_limit,
                                       bool saturated,
                                       double dt,
                                       bool implicit) {
    Eigen::Vector3d sigma = s;
    Eigen::Vector3d xi = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; ++i) {
        if (implicit) {
            const ImplicitSwitch resolved =
                resolveImplicitSwitch(s(i), w(i), gains.k1(i), gains.k2(i), dt, b(i));
            sigma(i) = resolved.sigma;
            xi(i) = resolved.xi;
        } else {
            xi(i) = (s(i) > 0.0) - (s(i) < 0.0);
        }
    }

    // Integral branch: the switching term plus the generalised linear one.
    const Eigen::Vector3d increment =
        -(gains.k2.cwiseProduct(xi) + gains.k4.cwiseProduct(s)) * dt;

    if (implicit) {
        integrateStaState(w, increment, w_limit, saturated);
    }

    // Proportional branch. The sqrt acts on the RESOLVED surface (which is s
    // itself under the explicit scheme), the linear term on the measured one:
    // k3*s is ordinary linear damping and has no discontinuity to resolve.
    Eigen::Vector3d u = Eigen::Vector3d::Zero();
    for (int i = 0; i < 3; ++i) {
        const double magnitude = std::sqrt(std::abs(sigma(i)));
        const double sign = (sigma(i) > 0.0) - (sigma(i) < 0.0);
        u(i) = -gains.k1(i) * magnitude * sign - gains.k3(i) * s(i) + w(i);
    }

    if (!implicit) {
        integrateStaState(w, increment, w_limit, saturated);
    }
    return u;
}

}  // namespace px4_offboard

#endif  // PX4_OFFBOARD_STA_REACHING_LAW_H
