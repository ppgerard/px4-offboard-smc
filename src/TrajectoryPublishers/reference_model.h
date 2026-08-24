#ifndef PX4_OFFBOARD_LOWLEVEL_REFERENCE_MODEL_H
#define PX4_OFFBOARD_LOWLEVEL_REFERENCE_MODEL_H

#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Eigen>

namespace landing {

// Third-order reference generator (roadmap item 11).
//
// WHAT IT REPLACES, and why the old form was not merely "a bit noisy".
//
// The guidance used to shape its velocity command with a first-order low-pass
// and then recover the acceleration by differencing it:
//
//     v_r[k] = alpha*u[k] + (1-alpha)*v_r[k-1]
//     a_r    = (v_r[k] - v_r[k-1]) / dt
//
// That difference is not a numerical derivative of a noisy signal -- it collapses
// EXACTLY, and the result is the problem:
//
//     a_r = (alpha/dt) * (u[k] - v_r[k-1])        = 20 * (u - v_r) at alpha=0.2, 100 Hz
//
// so the acceleration was PROPORTIONAL TO THE RAW, UNFILTERED command with a gain
// of 20. The filter smoothed the velocity and left the acceleration carrying every
// jump in the input. Since a_r enters the control law as m*a_r at full authority,
// a step in the steering estimate arrived in the force command multiplied by
// m*K_p*alpha/dt = 49.7 N per metre. The tag updates discretely, so those steps are
// not noise to be filtered -- they are how the measurement arrives. Measured in a
// steady Phase 1 hold: mean 0.026 N, peak 1.115 N, which is a 2.2 cm estimate jump.
//
// THE FIX is to make the acceleration a STATE rather than an output. Here the
// reference obeys
//
//     p_r_dot = v_r
//     v_r_dot = a_r
//     a_r_dot = j = -k1*a_r - k2*(v_r - v_cmd)
//
// with k1 = 2*zeta*omega and k2 = omega^2. A jump in v_cmd now moves the JERK; it
// cannot move a_r at all in that instant, at any bandwidth. A second-order model
// would still let a_r jump by omega^2 * delta, which is only an improvement while
// omega stays small -- the third state is what removes the dependence on tuning.
//
// The position integration itself stays where it was, in the phase logic: Phase 1
// re-anchors r_position to the vehicle (item 2b -- the reference must not run away
// while PX4 is flying) and Phase 2 integrates it open-loop. This class owns the two
// states that were previously reconstructed, not the guidance policy.
//
// ROS-free and simulator-free on purpose, like relative_state_filter.h: a reference
// generator is arithmetic, so its limits and its step response can be established
// offline in milliseconds. See test/reference_model_test.cpp.
struct ReferenceModelLimits {
  double max_velocity_xy = 0.0;      // [m/s]   norm over (x, y)
  double max_velocity_z = 0.0;       // [m/s]   magnitude
  double max_acceleration_xy = 0.0;  // [m/s^2] norm over (x, y)
  double max_acceleration_z = 0.0;   // [m/s^2] magnitude
  double max_jerk_xy = 0.0;          // [m/s^3] norm over (x, y)
  double max_jerk_z = 0.0;           // [m/s^3] magnitude
};

class ReferenceModel {
 public:
  // omega [rad/s] is the reference bandwidth; zeta the damping. omega <= 0 leaves
  // the model DISABLED and the caller on its previous path, which is what makes
  // this A/B-able against a single binary -- the pattern f_ext_observer_gain and
  // omega_ref_filter_hz already use.
  //
  // zeta = 1.0 by default and deliberately: overshoot in a velocity reference is
  // the guidance commanding the vehicle PAST the point it was asked to reach.
  void configure(double omega, double zeta = 1.0) {
    omega_ = std::max(0.0, omega);
    zeta_ = std::max(0.0, zeta);
    k1_ = 2.0 * zeta_ * omega_;
    k2_ = omega_ * omega_;
  }

  bool enabled() const { return omega_ > 0.0; }

  // Start from a known reference state rather than from whatever the states held
  // when the vehicle was last flown. Call it wherever the trajectory is seeded.
  void reset(const Eigen::Vector3d &velocity = Eigen::Vector3d::Zero(),
             const Eigen::Vector3d &acceleration = Eigen::Vector3d::Zero()) {
    velocity_ = velocity;
    acceleration_ = acceleration;
    jerk_.setZero();
  }

  // One step. Semi-implicit: the acceleration is advanced first and the NEW value
  // integrates the velocity, which is stable at this bandwidth where the explicit
  // form is only marginally so.
  void update(const Eigen::Vector3d &velocity_command,
              const ReferenceModelLimits &limits, double dt) {
    if (!enabled() || dt <= 0.0) {
      return;
    }

    jerk_ = -k1_ * acceleration_ - k2_ * (velocity_ - velocity_command);
    saturateXY(jerk_, limits.max_jerk_xy);
    saturateZ(jerk_, limits.max_jerk_z);

    acceleration_ += jerk_ * dt;
    const Eigen::Vector3d acceleration_free = acceleration_;
    saturateXY(acceleration_, limits.max_acceleration_xy);
    saturateZ(acceleration_, limits.max_acceleration_z);
    // Anti-windup: a clamped state must not keep being driven outward, or the
    // model has to unwind the excess before it can come off the bound. Same
    // reasoning as integrateStaState() and the f_ext clamp back-calculation.
    cancelOutwardDrive(acceleration_, acceleration_free, jerk_);

    velocity_ += acceleration_ * dt;
    const Eigen::Vector3d velocity_free = velocity_;
    saturateXY(velocity_, limits.max_velocity_xy);
    saturateZ(velocity_, limits.max_velocity_z);
    cancelOutwardDrive(velocity_, velocity_free, acceleration_);
  }

  const Eigen::Vector3d &velocity() const { return velocity_; }
  const Eigen::Vector3d &acceleration() const { return acceleration_; }
  // Available in closed form, which is the other half of what the third state
  // buys: the attitude reference's derivative no longer needs a second numerical
  // differentiation to exist.
  const Eigen::Vector3d &jerk() const { return jerk_; }

  double omega() const { return omega_; }
  double zeta() const { return zeta_; }

 private:
  static void saturateXY(Eigen::Vector3d &v, double max_norm) {
    if (max_norm <= 0.0) {
      return;
    }
    const double norm = std::hypot(v(0), v(1));
    if (norm > max_norm) {
      const double scale = max_norm / norm;
      v(0) *= scale;
      v(1) *= scale;
    }
  }

  static void saturateZ(Eigen::Vector3d &v, double max_value) {
    if (max_value <= 0.0) {
      return;
    }
    v(2) = std::clamp(v(2), -max_value, max_value);
  }

  // Where the state was clamped, remove the part of its driving term that points
  // further out. Sign-based per axis, which is enough: the clamp only ever
  // shrinks a component, never flips it.
  static void cancelOutwardDrive(const Eigen::Vector3d &clamped,
                                 const Eigen::Vector3d &unclamped,
                                 Eigen::Vector3d &drive) {
    for (int i = 0; i < 3; ++i) {
      if (clamped(i) != unclamped(i) && drive(i) * clamped(i) > 0.0) {
        drive(i) = 0.0;
      }
    }
  }

  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d jerk_ = Eigen::Vector3d::Zero();
  double omega_ = 0.0;
  double zeta_ = 1.0;
  double k1_ = 0.0;
  double k2_ = 0.0;
};

}  // namespace landing

#endif  // PX4_OFFBOARD_LOWLEVEL_REFERENCE_MODEL_H
