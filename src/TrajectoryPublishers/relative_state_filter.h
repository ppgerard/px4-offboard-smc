#ifndef RELATIVE_STATE_FILTER_H
#define RELATIVE_STATE_FILTER_H

// An extended Kalman filter for the vehicle state RELATIVE TO THE LANDING
// PLATFORM, updated directly from AprilTag corner pixels.
//
// It knows nothing about ROS, PX4, tf2 or AprilTag messages: numbers in, numbers
// out. That is what makes it testable without a simulator
// (test/relative_state_filter_test.cpp runs the consistency checks offline in a
// second), and what lets it run alongside the complementary filter it is meant to
// replace, so both can be scored against groundtruth in the SAME flight rather
// than across two runs in different wind.
//
//   x = [ p_rel (3)   vehicle body origin relative to the platform origin, world axes [m]
//         v_rel (3)   its derivative; on a static pad, the vehicle velocity   [m/s]
//         psi_pf(1)   platform yaw relative to the world x axis               [rad]
//         b_v   (3)   bias on the navigation velocity measurement             [m/s]
//         b_cam (3)   error in the camera's mounting offset, body axes         [m] ]
//
// b_cam is the calibration the filter used to ASSUME. r_camera_body is read off
// the airframe (or, here, the SDF) and is never exactly right; the error is fixed
// in metres, so in pixels it grows as 1/range -- 2 px at 3 m and 36 px at 0.3 m
// for a single centimetre. That is why it was worth an inflated R term
// (extrinsic_sigma) and why the inflation hurt most near the ground, discounting
// the tag in the one phase with no other lateral reference.
//
// Estimating it moves that uncertainty from R into P, where it belongs: it is one
// unknown constant, not fresh noise on every frame. It is observable for exactly
// the reason it hurts -- a descent sweeps the range by 10x, and a fixed metric
// offset changes its pixel signature across that sweep while a genuine position
// error does not. It is also expressed in BODY axes, so any yaw the vehicle turns
// separates it further from p_rel, which lives in world axes.
//
// b_v is a MEASUREMENT bias, not a force: it is what EKF2's velocity is wrong by,
// which is a slowly drifting quantity rather than white noise. It is NOT f_ext --
// that is an unmodelled input to the DYNAMICS and needs a specific-force input
// this filter is not given (roadmap item 8). The two look alike and sit on
// opposite sides of the filter.
//
// Modelling it replaces a fudge. Fusing a time-correlated velocity error at 50 Hz
// as if it were white drives the velocity covariance almost to zero, and the
// position covariance then grows far more slowly than the estimate really drifts.
// The previous compromise was an extra random walk laid directly on position,
// which grows as sqrt(t) where a real drifting bias grows LINEARLY -- so no single
// value was right both while the tag was visible and through a loss of it. With
// the bias as a state the uncertainty reaches position through the integration and
// comes out the right shape, with nothing to retune per outage length.
//
// f_ext (§05 / roadmap item 8) and the camera boresight bias are deliberately
// left out until this much is consistent: every extra state is another thing
// that can absorb an error that belongs somewhere else, and a filter whose NIS
// does not match its covariance cannot tell you which.
//
// WHY PIXELS RATHER THAN THE PnP POSE
//
// A pose measurement forces an invented R(range, viewing angle): pose noise is
// neither constant nor isotropic, it grows with range in the along-axis
// direction and depends on how obliquely the board is seen, so any R written for
// it is a fit. Pixel noise really is roughly constant and isotropic (~0.5-1 px,
// and measurable from reprojection residuals), and the range scaling then falls
// out of the projection Jacobian instead of being asserted. It also removes the
// planar ambiguity structurally rather than filtering around it -- the ambiguity
// is a property of resolving a POSE from a near-planar target, and no pose is
// resolved here -- degrades gracefully when only part of the board is visible
// (each corner is its own two rows), and lets a tag too small for a stable pose
// still contribute what it does know.
//
// WHAT THE §03 DEFECTS WERE, AND WHERE THEY ARE FIXED HERE
//
//  - Stale measurements applied as current (P0). A tag measurement is 40-100 ms
//    old by the time it is processed; forming its innovation against the state
//    NOW makes the lag look like innovation and biases the estimate along the
//    direction of travel. Fixed by a ring buffer of states and events
//    (history_): a measurement stamped t_m rewinds the filter to t_m, applies
//    the update there, and replays everything that has happened since. Replay is
//    exact -- no Larsen-style approximation -- because with 7 states and ~100 ms
//    of 100 Hz events it costs microseconds, and an exact rewind is one less
//    thing to blame when the NIS statistics come out wrong.
//
//  - Fixed gain, no outlier rejection, no confidence. fusion_alpha_ = 0.1 was
//    constant while tag noise grows with range; here the gain is P H' S^-1, the
//    chi-squared gate comes free from S, and the covariance is the confidence
//    signal the descent gate (§06) and the failsafe (§07) both want.
//
// CONVENTIONS
//   - Time is a plain double in seconds on a single monotonic clock. The caller
//     is responsible for putting measurement stamps on that clock; the filter
//     only ever compares times it has been given.
//   - Rotations follow the rest of this package: R_world_body maps body (FLU) to
//     world (ENU), R_body_camera maps camera optical to body.
//   - The platform frame is the one the detector's object points are written in
//     (see pose_estimation.cpp): x/y in the marker plane, z up out of it, origin
//     at the platform centre. psi_pf is its yaw in world axes, so a corner at
//     platform coordinates c sits at Rz(psi_pf) * c in world axes.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <memory>
#include <vector>

#include <eigen3/Eigen/Eigen>

namespace landing {

// Pinhole intrinsics, in the same units the CameraInfo K matrix uses. SITL's
// camera has no distortion; a real lens needs the corners undistorted before
// they get here, which is the detector's job and not this filter's.
struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;

  bool valid() const { return fx > 0.0 && fy > 0.0; }
};

// One tag's worth of corner correspondences, already matched: pixels[i] is the
// image of points_platform[i]. Fewer than four is fine and so is more than four
// (both tags in one update) -- the filter only ever sees a list of pairs, which
// is exactly why partial visibility degrades gracefully.
//
// The attitude is the vehicle attitude AT THE MEASUREMENT'S OWN TIMESTAMP, not
// the latest available (item 3's lesson; see attitudeAt() in
// landing_trajectory_base.h). The extrinsics are passed in rather than
// configured so that the filter has no calibration state of its own.
struct CornerObservation {
  std::vector<Eigen::Vector2d> pixels;
  std::vector<Eigen::Vector3d> points_platform;
  Eigen::Matrix3d R_world_body = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d R_body_camera = Eigen::Matrix3d::Identity();
  Eigen::Vector3d r_camera_body = Eigen::Vector3d::Zero();
  CameraIntrinsics camera;

  bool valid() const {
    return camera.valid() && !pixels.empty() && pixels.size() == points_platform.size();
  }
};

// ---- Rangefinder geometry ---------------------------------------------------
// Where the raw beam becomes an altitude. Kept here, next to the filter and free
// of ROS, because it is arithmetic with a right answer and it is worth being able
// to check it offline: the same lever arm was silently zero inside EKF2 for the
// whole project and cost 14.5 cm of altitude bias before anyone measured it.
//
// The beam looks along body -z. With the vehicle tilted, the range it reports is
// the slant distance, and the sensor is not at the body origin, so
//
//     h = range * cos(tilt) - (R_world_body * r_sensor_body)_z
//
// where cos(tilt) is just R_world_body(2,2), the world-vertical component of the
// body z axis. Level, with the mount 0.145 m below the origin, that is
// range + 0.145 -- which is why the beam reads 0.155 m with the body at 0.30 m,
// and why it goes blind (0.1 m minimum) at 0.245 m of body height.
//
// This assumes flat ground under the vehicle, which is a project constraint (a
// static, flat pad) rather than an approximation being smuggled in. Off level the
// assumption decays -- the beam lands somewhere else on the pad -- so the tilt
// limit below is a real gate, not a formality.
struct RangeGeometry {
  Eigen::Vector3d r_sensor_body{0.10, 0.0, -0.145};  // sensor in body FLU [m]
  double max_tilt = 0.35;      // [rad] beyond ~20 deg the flat-ground assumption goes
  double sigma_base = 0.02;    // [m]   fixed part of the beam's error
  double sigma_scale = 0.01;   // [m/m] proportional part, 1% of range
  double sigma_lever = 0.01;   // [m]   how well the mount position is known
};

struct RangeAltitude {
  bool valid = false;
  double altitude = 0.0;  // [m] body origin above the pad
  double sigma = 0.0;     // [m] one standard deviation of the above
  double cos_tilt = 1.0;
};

// Convert one beam reading into an altitude of the body origin, with its sigma.
// `attitude_sigma` is the navigation attitude error, which enters because a
// tilted beam turns attitude error into range error: d(h)/d(tilt) = -range*sin(tilt),
// zero at level and growing as the vehicle leans, which is exactly the behaviour
// a fixed sigma would get wrong in both directions.
inline RangeAltitude rangeToAltitude(double range, const Eigen::Matrix3d &R_world_body,
                                     const RangeGeometry &geometry, double attitude_sigma) {
  RangeAltitude result;
  result.cos_tilt = R_world_body(2, 2);
  if (!std::isfinite(range) || range <= 0.0 || result.cos_tilt < std::cos(geometry.max_tilt)) {
    return result;
  }
  const double lever_z = (R_world_body * geometry.r_sensor_body)(2);
  result.altitude = range * result.cos_tilt - lever_z;

  const double sin_tilt =
      std::sqrt(std::max(0.0, 1.0 - result.cos_tilt * result.cos_tilt));
  const double beam = geometry.sigma_base + geometry.sigma_scale * range;
  const double tilt_term = range * sin_tilt * attitude_sigma;
  result.sigma = std::sqrt(beam * beam + tilt_term * tilt_term +
                           geometry.sigma_lever * geometry.sigma_lever);
  result.valid = true;
  return result;
}

// The largest body altitude consistent with the beam being INSIDE its minimum
// range -- i.e. what the silence in the blind zone actually says.
inline double blindZoneCeiling(double min_range, const Eigen::Matrix3d &R_world_body,
                               const RangeGeometry &geometry) {
  const double lever_z = (R_world_body * geometry.r_sensor_body)(2);
  return min_range * R_world_body(2, 2) - lever_z;
}

// ---- The tag-loss ladder (roadmap item 5, §07) ------------------------------
//
// Kept here, ROS-free and beside the filter, because its inputs ARE the filter's
// outputs and because a failsafe that can only be exercised by a three-minute
// SITL flight is a failsafe nobody exercises. The node supplies the numbers; this
// decides the tier, and test/relative_state_filter_test.cpp checks it in a
// millisecond.
//
// Two of the inputs have no clock equivalent, which is the whole reason item 5
// was ordered after item 7:
//
//   - `age` is the age of the last measurement the chi-squared gate ACCEPTED, so
//     a stream of REJECTED detections reads as a lost tag rather than a healthy
//     one. An occlusion and a mis-decode are different failures and a TF
//     timestamp cannot tell them apart.
//   - `sigma_xy` says what that staleness is worth in metres. It is compared
//     against the two lengths that actually bound the decisions being made: the
//     descent cone at the current height, and the patch of ground the camera can
//     see from it. A fixed age cannot do that -- the cone runs from 0.95 m at 3 m
//     to 0.14 m at the commit altitude while sigma grows the same way regardless,
//     so one number cannot mean the same thing at both ends.
enum class TagLossTier { kCoast = 0, kHold = 1, kReacquire = 2, kAbort = 3 };

struct TagLossThresholds {
  double coast_min_seconds = 0.3;   // below this it is a blink; do nothing [s]
  // Above this, stop descending whatever the cone says. It has to sit BELOW
  // reacquire_seconds or the Hold rung is unreachable: the ladder would step
  // straight from Coast to a climb, which is the one response that throws away
  // the approach's alignment. Caught by the offline case rather than in flight.
  double coast_max_seconds = 1.0;   // [s]
  double reacquire_seconds = 1.5;   // §07's climb threshold [s]
  double abort_seconds = 5.0;       // §07's abort threshold [s]
  // How much of the covariance to believe. Two sigma, because the covariance is
  // measured OPTIMISTIC in exactly this regime: the offline blackout case has the
  // error reaching ~2.1 sigma while dead reckoning, the drift being correlated
  // where the filter models it as a random walk.
  double sigma_margin = 2.0;
  double cone_slope = 0.30;        // [m/m] must match the guidance cone
  double cone_radius_min = 0.05;   // [m]
  double footprint_slope = 1.19;   // [m/m] ground radius the camera sees per metre of height
  // Below this height the ladder cannot escalate past Hold. Both markers leave
  // the frame in the last ~25 cm of EVERY landing, so the terminal phase is
  // permanently "tag lost" by construction: a climb there is a manoeuvre in
  // ground effect on a target that was never going to be visible, and an abort
  // there throws away a landing that is already all but complete.
  double no_escalation_below = 0.30;  // [m], the commit altitude
  int max_attempts = 3;               // §07
};

struct TagLossInputs {
  double age = 0.0;         // [s] since the last ACCEPTED measurement; may be infinite
  double sigma_xy = 0.0;    // [m] estimator XY standard deviation; 0 when unavailable
  double height = 0.0;      // [m] above the pad
  double xy_error = 0.0;    // [m] estimated horizontal offset from the pad
  int attempts_used = 0;    // reacquire climbs already spent this landing
  bool ever_acquired = false;
};

inline TagLossTier tagLossTier(const TagLossInputs &in, const TagLossThresholds &t) {
  // Before the first detection the aircraft is SEARCHING, not coasting: the age
  // is infinite by construction and the response belongs to the approach phase,
  // which flies a bounded odometry hold point and looks. Running the ladder here
  // would abort every ordinary run a few seconds after it started.
  if (!in.ever_acquired) {
    return TagLossTier::kCoast;
  }

  const double height = in.height > 0.0 ? in.height : 0.0;
  const double cone_radius = t.cone_slope * height + t.cone_radius_min;
  const double footprint_radius = t.footprint_slope * height;
  // Where the aircraft might be, rather than where it is claimed to be.
  const double reach = in.xy_error + t.sigma_margin * in.sigma_xy;

  TagLossTier tier = TagLossTier::kCoast;
  if (in.age >= t.coast_min_seconds) {
    // Past a blink. Whether coasting is still safe is a question about the
    // descent this estimate is steering: keep coasting while the aircraft can
    // still be asserted, to sigma_margin sigma, to be inside the cone it is
    // descending through. That tightens on its own as the cone narrows.
    const bool inside_cone_with_confidence = reach < cone_radius;
    if (in.age >= t.coast_max_seconds || !inside_cone_with_confidence) {
      tier = TagLossTier::kHold;
    }
  }

  // Climbing only helps when the tag CANNOT be in frame from here. If it is
  // inside the footprint and simply not being decoded, altitude buys nothing and
  // the climb costs the approach its alignment.
  if (tier == TagLossTier::kHold) {
    const bool out_of_footprint = reach > footprint_radius;
    if (in.age >= t.reacquire_seconds || out_of_footprint) {
      tier = TagLossTier::kReacquire;
    }
  }

  if (tier == TagLossTier::kReacquire &&
      (in.age >= t.abort_seconds || in.attempts_used > t.max_attempts)) {
    tier = TagLossTier::kAbort;
  }

  if (height < t.no_escalation_below && tier > TagLossTier::kHold) {
    tier = TagLossTier::kHold;
  }
  return tier;
}

struct RelativeStateFilterConfig {
  // ---- Process noise --------------------------------------------------------
  // The filter is fed no acceleration input, so the whole vehicle acceleration is
  // unmodelled and this has to cover it: the T2 commands up to 2.5 m/s^2 in the
  // approach and rides 5 m/s gusts.
  double accel_noise_density = 2.0;          // [m/s^2/sqrt(Hz)]
  // How fast the navigation velocity's bias is allowed to wander. Small, because
  // EKF2's velocity error drifts over seconds rather than jumping, and because the
  // state is only observable while the tag is pinning position -- through a tag
  // loss this is what decides how fast the filter admits it is dead reckoning.
  // Position random walk, standing in for the fact that the navigation velocity's
  // error is time-correlated rather than white. A random walk is the wrong SHAPE
  // for that -- real drift grows linearly with an outage, this grows as its square
  // root -- so it is a compromise, and b_v below is the principled replacement.
  //
  // It is still the shipped default, on flight evidence. See the note on b_v.
  double position_noise_density = 0.02;  // [m/sqrt(s)]

  // Velocity-measurement bias, the principled version of the term above: model the
  // correlated part as a state and the uncertainty reaches position through the
  // integration with the right shape. OFF by default (zero process noise pins it
  // at its initial value of zero), and that is a flight result rather than a
  // preference.
  //
  // Offline it is clearly better: NEES 2.7 against 6.1, and a 4 s tag loss bounded
  // at 1.3 sigma against 2.5. In flight it made the APPROACH slightly better and
  // the terminal phase clearly worse -- synthetic commit 1.08 -> 2.12 cm and
  // touchdown 2.74 -> 5.61 cm, with the same direction on the real camera. The
  // mechanism is visible in the numbers: near the ground R explodes as 1/range, so
  // the tag stops correcting, b_v stops being observable, and whatever value it
  // last took is then integrated as truth through exactly the phase that has no
  // other XY reference. The offline suite cannot see this because it has no
  // terminal phase where the measurement is discounted rather than absent.
  //
  // To finish it: add that case to the offline suite, then either freeze b_v when
  // the tag is not effectively correcting, or bound it far more tightly than
  // velocity_bias_max. Turn on with velocity_bias_noise_density > 0.
  double velocity_bias_noise_density = 0.0;  // [m/s/sqrt(s)]
  // The pad does not move, so this is not "how fast can the platform turn" but
  // "how fast may the estimate of its heading wander". It has to be small: once
  // attitude uncertainty is in R (below), a rotation of the whole corner pattern
  // about the boresight looks like measurement noise, so psi_pf is only weakly
  // observed -- and at 0.01 it random-walked to 76 degrees in flight, dragged the
  // reprojection with it, and locked the gate out. Its accuracy is bounded by the
  // navigation yaw error either way, which is a bound, not a drift.
  double platform_yaw_noise_density = 0.001;  // [rad/sqrt(s)]

  // ---- Measurement noise ----------------------------------------------------
  double pixel_sigma = 1.0;      // [px]   corner localisation, measurable from residuals
  // The attitude handed in with each observation is EKF2's, not truth, and its
  // error is COMMON to every corner in the frame: it swings the whole bearing
  // bundle together, which is indistinguishable from the vehicle having moved
  // sideways. At a 539 px focal length a single degree is ~9 px, so this term is
  // an order of magnitude larger than the corner noise and leaving it out is what
  // makes a filter that looks consistent in innovation and is badly overconfident
  // in state -- measured in flight at 2-6 sigma of position error before it was
  // modelled. It enters as a correlated block in R rather than as more diagonal:
  // that keeps the tag's scale and yaw information sharp while loosening only the
  // lateral information it genuinely cannot pin down.
  double attitude_sigma = 0.010;  // [rad] per axis, EKF2 attitude
  // How well the camera's position on the airframe is known. The same structure
  // as the attitude term and for the same reason -- it is common to every corner
  // -- but it matters most at the other end of the flight: a fixed metric error
  // is a fixed pixel error divided by range, so a centimetre of lever arm is 2 px
  // at 3 m and 36 px at 0.3 m. Leaving it out makes the gate tightest exactly
  // where the model is weakest, and it was measured doing so: 21% of perfectly
  // good measurements rejected during a descent. This project has an open
  // question of about 4 cm on camera_offset_body.z, so this is not hypothetical.
  double extrinsic_sigma = 0.02;  // [m]
  // Only the WHITE part of the navigation velocity error now: the correlated part
  // is a state (b_v), so this no longer has to be inflated to cover it.
  double velocity_sigma = 0.15;  // [m/s]  inflated for time correlation while b_v is off
  double range_sigma = 0.03;     // [m]    dist_bottom vs groundtruth: mean 3 mm, worst 1.9 cm
  double pose_sigma = 0.10;      // [m]    debug pose mode only

  // The pipeline delay this filter cannot see. Where the detector stamps on
  // another clock, only the VARIABLE part of the latency is recoverable and the
  // constant floor is not, so a measurement is older than its stamp says by an
  // unknown amount. That turns into a common-mode position error of speed times
  // delay -- which is why it enters R as a rank-one term along the current
  // velocity rather than as a bigger number everywhere: it costs nothing in the
  // hover and correctly loosens the measurement on a moving approach.
  double latency_sigma = 0.03;   // [s]

  // ---- Gating ---------------------------------------------------------------
  double gate_probability = 0.999;  // chi-squared gate on the normalised innovation
  // A filter that has just been initialised knows nothing about the platform yaw
  // and little about where it is, so its first innovations are legitimately huge
  // -- gate them and it can never converge. Accept this many measurements
  // ungated, which is the standard bootstrap and also the recovery path if the
  // platform-frame convention turns out to be rotated by a right angle.
  int bootstrap_updates = 5;
  // Sustained rejection means the covariance is wrong, not that the world is:
  // inflate and let the measurements back in rather than sit there confidently
  // lost. Timed, not counted -- a counter resets on any accepted measurement, so
  // an occasional weak one that still passes keeps a genuinely lost filter locked
  // out forever. Measured in flight: rejection climbed to 80% and the estimate
  // never recovered, because the small tag kept resetting the count while the
  // large one was gated out.
  double reject_recovery_seconds = 1.0;
  double reject_inflation_factor = 4.0;

  // ---- Bookkeeping ----------------------------------------------------------
  double history_seconds = 1.0;    // rewind depth for delayed measurements
  double max_step_seconds = 0.05;  // a predict longer than this is split
  double min_depth = 0.10;         // [m] corners closer than this along the boresight are refused

  // ---- Initial covariance ---------------------------------------------------
  double initial_position_sigma = 2.0;  // [m]
  double initial_velocity_sigma = 0.5;       // [m/s]
  // Zero while b_v is off: an initial uncertainty on a state with no process noise
  // would let it be estimated once, early, and then held forever.
  double initial_velocity_bias_sigma = 0.0;  // [m/s]
  // Hard bound on the velocity bias, applied after every update. See the note at
  // the clamp: an unbounded bias state turns a large position innovation into a
  // runaway.
  double velocity_bias_max = 0.30;  // [m/s]

  // ---- Camera mounting bias (b_cam) -----------------------------------------
  // On by default: unlike b_v, this one is a fixed physical constant rather than a
  // drifting quantity, so it cannot be integrated into a runaway, and the term it
  // replaces in R is the one measured to be dominating the close-range
  // conservatism.
  bool estimate_camera_bias = true;
  // Per-axis, and only the boresight is non-zero -- because only the boresight is
  // OBSERVABLE on this mission. Measured offline, and it inverts the intuition:
  //
  //   - The LATERAL offset (x, y) shifts the whole image sideways, which is
  //     exactly what a p_rel error does. Nothing separates them but vehicle
  //     rotation, and a landing barely rotates: yaw is pinned to the platform
  //     heading by project constraint and the tilt in wind is a few degrees. With
  //     +-5 deg of yaw the state does not converge at all, and it absorbs position
  //     error while failing to (5.1 cm vs 4.5 cm below 0.5 m). Even +-57 deg of
  //     deliberate yaw only recovers about a third of it.
  //   - The BORESIGHT offset (z) changes the RANGE to the pad, and therefore the
  //     apparent SCALE of the marker -- and the altitude is independently pinned
  //     by the rangefinder. So "the beam says 1.2 m but the tag looks like 1.25 m"
  //     is information about the mounting and nothing else. It converges to within
  //     1 mm of a 2 cm error, with no rotation needed.
  //
  // Which is a pleasing result: the boresight component is the one this project
  // actually had an open question about (camera_offset_body.z), and it is the one
  // the rangefinder makes identifiable. Range diversity, incidentally, does NOT
  // help -- a descent scales the pixel effect of a mounting error and a position
  // error identically, which is where the original argument for this state was
  // wrong.
  Eigen::Vector3d initial_camera_bias_sigma{0.0, 0.0, 0.03};  // [m]
  // Nearly zero: the camera does not move on the airframe. Non-zero only so the
  // state can still respond slowly on a long flight rather than freezing on an
  // early, badly conditioned estimate.
  double camera_bias_noise_density = 0.0005;  // [m/sqrt(s)]
  // A bound, for the same reason b_v has one: a state that enters the measurement
  // model can absorb an error that belongs elsewhere. Nothing on this airframe is
  // mounted 10 cm from where the SDF says.
  double camera_bias_max = 0.10;  // [m]
  // What R still has to carry once b_cam is a state: the part of the extrinsic
  // error three translation states cannot represent, chiefly boresight rotation.
  double extrinsic_residual_sigma = 0.005;  // [m]
  double initial_yaw_sigma = 0.6;       // [rad]
};

class RelativeStateFilter {
 public:
  static constexpr int kStateDim = 13;
  using StateVector = Eigen::Matrix<double, kStateDim, 1>;
  using StateMatrix = Eigen::Matrix<double, kStateDim, kStateDim>;

  // Why a measurement did or did not change the estimate. Callers use this for
  // the health signal: a measurement that was REJECTED is not evidence the tag
  // was seen, which is the distinction tagAge() alone could never make.
  enum class Result {
    kApplied,
    kRejected,        // failed the chi-squared gate
    kNotInitialized,
    kTooOld,          // stamped before the ring buffer reaches
    kInvalid          // unusable geometry: behind the camera, no intrinsics, empty
  };

  explicit RelativeStateFilter(const RelativeStateFilterConfig &config = {})
      : config_(config), gate_z_(normalQuantile(config.gate_probability)) {
    state_.setZero();
    covariance_.setIdentity();
  }

  void setConfig(const RelativeStateFilterConfig &config) {
    config_ = config;
    gate_z_ = normalQuantile(config.gate_probability);
  }
  const RelativeStateFilterConfig &config() const { return config_; }

  void initialize(double time, const Eigen::Vector3d &position_relative,
                  const Eigen::Vector3d &velocity_relative, double platform_yaw,
                  const Eigen::Vector3d &velocity_bias = Eigen::Vector3d::Zero(),
                  const Eigen::Vector3d &camera_bias = Eigen::Vector3d::Zero()) {
    state_.setZero();
    state_.segment<3>(0) = position_relative;
    state_.segment<3>(3) = velocity_relative;
    state_(6) = wrapToPi(platform_yaw);

    covariance_.setZero();
    covariance_.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() * config_.initial_position_sigma * config_.initial_position_sigma;
    covariance_.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() * config_.initial_velocity_sigma * config_.initial_velocity_sigma;
    covariance_(6, 6) = config_.initial_yaw_sigma * config_.initial_yaw_sigma;
    state_.segment<3>(7) = velocity_bias;
    covariance_.block<3, 3>(7, 7) = Eigen::Matrix3d::Identity() *
                                    config_.initial_velocity_bias_sigma *
                                    config_.initial_velocity_bias_sigma;
    state_.segment<3>(10) = camera_bias;
    covariance_.block<3, 3>(10, 10) = cameraBiasPrior().array().square().matrix().asDiagonal();

    time_ = time;
    initialized_ = true;
    history_.clear();
    applied_count_ = 0;
    rejected_count_ = 0;
    consecutive_rejects_ = 0;
    bootstrap_remaining_ = config_.bootstrap_updates;
    last_accepted_time_ = time;
    nis_normalised_sum_ = 0.0;
    nis_samples_ = 0;
    last_nis_ = 0.0;
    last_nis_dof_ = 0;
    last_gate_threshold_ = 0.0;
  }

  bool initialized() const { return initialized_; }
  double time() const { return time_; }

  // ---- Propagation -----------------------------------------------------------
  // Called at the control rate. Recorded in the ring buffer so a late measurement
  // can rewind through it.
  void predict(double time) {
    if (!initialized_ || time <= time_) {
      return;
    }
    Event event;
    event.kind = Event::Kind::kPredict;
    event.time = time;
    submit(std::move(event));
  }

  // ---- Measurements ----------------------------------------------------------
  // All of these take the time the measurement was MADE, not the time it arrived.
  // A stamp in the past rewinds; a stamp in the future is treated as now.

  // The bring-up measurement: h = p_rel, H linear. Keeps the whole rewind /
  // gating / covariance path exercisable against the behaviour the complementary
  // filter already has, with the projection geometry taken out of the picture.
  Result addPose(double time, const Eigen::Vector3d &position_relative) {
    Event event;
    event.kind = Event::Kind::kPose;
    event.time = time;
    event.vector = position_relative;
    return submit(std::move(event));
  }

  Result addCorners(double time, const CornerObservation &observation) {
    if (!observation.valid()) {
      return Result::kInvalid;
    }
    Event event;
    event.kind = Event::Kind::kCorners;
    event.time = time;
    event.corners = std::make_shared<const CornerObservation>(observation);
    return submit(std::move(event));
  }

  // Vehicle velocity in world axes, from the navigation solution. On a static pad
  // this measures v_rel directly, and it is what keeps the velocity state from
  // being a free parameter that the position has to pay for.
  Result addVelocity(double time, const Eigen::Vector3d &velocity_world) {
    Event event;
    event.kind = Event::Kind::kVelocity;
    event.time = time;
    event.vector = velocity_world;
    return submit(std::move(event));
  }

  // Height of the body origin above the pad, from the rangefinder-aided terrain
  // estimate. Ungated: it is the altitude reference the rest of the stack already
  // trusts, and gating it against a covariance this filter derived would let a
  // drifting estimate lock the good measurement out.
  // `sigma` is per-measurement because the beam's error is not constant: it grows
  // with range and with tilt (see rangeToAltitude()). Pass a non-positive value to
  // fall back on the configured range_sigma.
  Result addRangeAltitude(double time, double altitude, double sigma = -1.0) {
    Event event;
    event.kind = Event::Kind::kRangeAltitude;
    event.time = time;
    event.scalar = altitude;
    event.scalar_low = sigma > 0.0 ? sigma : config_.range_sigma;
    return submit(std::move(event));
  }

  // The blind zone as information rather than as a gap.
  //
  // Below ~0.245 m of body height the beam is inside its minimum range and stops
  // measuring -- but that silence is a statement, and a two-sided one: the vehicle
  // is within 0.245 m of the pad, AND it is not underneath it. Nothing used to use
  // either half, so the last 30 cm dead-reckoned on a GPS-referenced z and drifted
  // both ways -- one stalled run reported 0.338 m after descending for 8 s from
  // 0.300 m, and a real-camera landing reported -1.13 m while sitting on the pad,
  // which is what stopped its touchdown check from ever firing.
  //
  // Applied as a PROJECTION rather than as a measurement, and only on the side
  // that is violated: the state is moved onto the feasible set through the
  // covariance -- so the velocity moves with it -- and the covariance itself is
  // left alone, because an interval is not evidence about how well the altitude is
  // known. That choice matters twice over. It is idempotent, so it needs no rate
  // limit and cannot double-count; and a rate-limited soft version does not
  // actually hold, which was measured: against a navigation velocity insisting on
  // -0.14 m/s the estimate settled 0.14 m underground between pulls, which is the
  // whole failure it exists to prevent.
  Result addAltitudeBounds(double time, double floor, double ceiling) {
    Event event;
    event.kind = Event::Kind::kAltitudeBounds;
    event.time = time;
    event.scalar = ceiling;
    event.scalar_low = floor;
    return submit(std::move(event));
  }

  // ---- Outputs ---------------------------------------------------------------
  const StateVector &state() const { return state_; }
  const StateMatrix &covariance() const { return covariance_; }
  Eigen::Vector3d positionRelative() const { return state_.segment<3>(0); }
  Eigen::Vector3d velocityRelative() const { return state_.segment<3>(3); }
  double platformYaw() const { return state_(6); }
  Eigen::Vector3d velocityBias() const { return state_.segment<3>(7); }
  // The mounting error the filter has learned, and how sure it is of it. Published,
  // because it is a calibration output: it is the number a bench measurement of the
  // camera mount should agree with.
  Eigen::Vector3d cameraBias() const { return state_.segment<3>(10); }
  Eigen::Vector3d cameraBiasStdDev() const {
    return Eigen::Vector3d(std::sqrt(std::max(covariance_(10, 10), 0.0)),
                           std::sqrt(std::max(covariance_(11, 11), 0.0)),
                           std::sqrt(std::max(covariance_(12, 12), 0.0)));
  }

  Eigen::Vector3d positionStdDev() const {
    return Eigen::Vector3d(std::sqrt(std::max(covariance_(0, 0), 0.0)),
                           std::sqrt(std::max(covariance_(1, 1), 0.0)),
                           std::sqrt(std::max(covariance_(2, 2), 0.0)));
  }
  double platformYawStdDev() const { return std::sqrt(std::max(covariance_(6, 6), 0.0)); }

  // Normalised innovation squared of the last gated measurement, its degrees of
  // freedom and the gate it was compared against. If the covariance is right,
  // lastNIS() averages its dof -- that claim is the whole basis for trusting any
  // accuracy number this filter produces, so it is published, not just logged.
  // RMS reprojection residual of the last accepted corner update [px], and the
  // RMS of what S says it should have been. Together these are how pixel_sigma
  // gets MEASURED instead of assumed: pixel noise is the part of the residual that
  // varies frame to frame, and the ratio of the two says whether R is honest.
  double lastResidualRms() const { return last_residual_rms_; }
  double lastResidualPredictedRms() const { return last_residual_predicted_rms_; }
  double lastNIS() const { return last_nis_; }
  int lastNISDof() const { return last_nis_dof_; }
  double lastGateThreshold() const { return last_gate_threshold_; }
  double meanNormalisedNIS() const {
    return nis_samples_ > 0 ? nis_normalised_sum_ / static_cast<double>(nis_samples_) : 0.0;
  }
  long appliedCount() const { return applied_count_; }
  long rejectedCount() const { return rejected_count_; }
  int consecutiveRejects() const { return consecutive_rejects_; }
  double rejectFraction() const {
    const long total = applied_count_ + rejected_count_;
    return total > 0 ? static_cast<double>(rejected_count_) / static_cast<double>(total) : 0.0;
  }

  // Predicted pixels for an observation, given the current state. Exposed for the
  // reprojection-residual diagnostics that are how pixel_sigma gets measured
  // rather than guessed.
  bool projectCorners(const CornerObservation &observation,
                      std::vector<Eigen::Vector2d> &predicted) const {
    Eigen::VectorXd innovation;
    Eigen::MatrixXd H;
    std::vector<Eigen::Vector2d> projected;
    if (!cornerModel(state_, observation, innovation, H, projected)) {
      return false;
    }
    predicted = projected;
    return true;
  }

  // Innovation and Jacobian of an observation against the current state. Public
  // because the numerical-differentiation check in the unit test is the only
  // thing standing between a sign error in H and a filter that gates every good
  // measurement out while looking perfectly healthy from outside.
  bool cornerModel(const CornerObservation &observation, Eigen::VectorXd &innovation,
                   Eigen::MatrixXd &H, Eigen::MatrixXd *attitude_jacobian = nullptr,
                   Eigen::MatrixXd *extrinsic_jacobian = nullptr) const {
    std::vector<Eigen::Vector2d> projected;
    return cornerModel(state_, observation, innovation, H, projected, attitude_jacobian,
                       extrinsic_jacobian);
  }

 private:
  struct Event {
    enum class Kind { kPredict, kPose, kCorners, kVelocity, kRangeAltitude, kAltitudeBounds };

    Kind kind = Kind::kPredict;
    double time = 0.0;
    Eigen::Vector3d vector = Eigen::Vector3d::Zero();
    double scalar = 0.0;      // range altitude, or the upper bound
    double scalar_low = 0.0;  // lower bound
    int applied_side = 0;     // which bound a replay must repeat: -1 low, +1 high
    std::shared_ptr<const CornerObservation> corners;

    // Whether this event changed the state when it was first submitted. A replay
    // repeats the decision rather than re-taking it: re-gating against a
    // covariance that has since moved would make the estimate depend on the order
    // measurements happened to arrive in, which is precisely what the rewind
    // exists to remove.
    bool was_applied = false;

    // The filter immediately BEFORE this event, which is what a rewind restores.
    StateVector state_before = StateVector::Zero();
    StateMatrix covariance_before = StateMatrix::Zero();
    double time_before = 0.0;
  };

  // ---- Time ordering and the ring buffer -------------------------------------
  Result submit(Event &&event) {
    if (!initialized_) {
      return Result::kNotInitialized;
    }
    if (event.time >= time_) {
      return applyAndRecord(std::move(event), /*is_replay=*/false);
    }
    // Out of order: the measurement belongs before something already applied.
    if (history_.empty() || event.time < history_.front().time_before) {
      return Result::kTooOld;
    }
    std::deque<Event> tail = rewindTo(event.time);
    const Result result = applyAndRecord(std::move(event), /*is_replay=*/false);
    for (Event &pending : tail) {
      applyAndRecord(std::move(pending), /*is_replay=*/true);
    }
    return result;
  }

  // Restore the filter to just before the first event later than `time`, and hand
  // back the events that have to be replayed afterwards.
  std::deque<Event> rewindTo(double time) {
    std::deque<Event> tail;
    auto first = history_.begin();
    while (first != history_.end() && first->time <= time) {
      ++first;
    }
    if (first == history_.end()) {
      return tail;
    }
    state_ = first->state_before;
    covariance_ = first->covariance_before;
    time_ = first->time_before;
    tail.assign(std::make_move_iterator(first), std::make_move_iterator(history_.end()));
    history_.erase(first, history_.end());
    return tail;
  }

  Result applyAndRecord(Event &&event, bool is_replay) {
    event.state_before = state_;
    event.covariance_before = covariance_;
    event.time_before = time_;

    predictTo(event.time);

    Result result = Result::kApplied;
    switch (event.kind) {
      case Event::Kind::kPredict:
        break;
      case Event::Kind::kPose:
        result = applyPose(event, is_replay);
        break;
      case Event::Kind::kCorners:
        result = applyCorners(event, is_replay);
        break;
      case Event::Kind::kVelocity:
        result = applyVelocity(event);
        break;
      case Event::Kind::kRangeAltitude:
        result = applyRangeAltitude(event);
        break;
      case Event::Kind::kAltitudeBounds:
        result = applyAltitudeBounds(event, is_replay);
        break;
    }
    event.was_applied = (result == Result::kApplied);

    history_.push_back(std::move(event));
    trimHistory();
    return result;
  }

  void trimHistory() {
    const double horizon = time_ - config_.history_seconds;
    while (history_.size() > 1 && history_.front().time < horizon) {
      history_.pop_front();
    }
  }

  // ---- Prediction ------------------------------------------------------------
  void predictTo(double time) {
    double remaining = time - time_;
    if (remaining <= 0.0) {
      return;
    }
    while (remaining > 0.0) {
      const double step = std::min(remaining, config_.max_step_seconds);
      predictStep(step);
      remaining -= step;
    }
    time_ = time;
  }

  // Constant velocity with white acceleration noise; the platform yaw is a random
  // walk because the pad does not move but the estimate of its heading may still
  // wander with viewing geometry.
  void predictStep(double dt) {
    state_.segment<3>(0) += state_.segment<3>(3) * dt;

    StateMatrix F = StateMatrix::Identity();
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

    const double qa = config_.accel_noise_density * config_.accel_noise_density;
    const double qb = config_.velocity_bias_noise_density * config_.velocity_bias_noise_density;
    const double qp = config_.position_noise_density * config_.position_noise_density;
    const double qpsi = config_.platform_yaw_noise_density * config_.platform_yaw_noise_density;

    StateMatrix Q = StateMatrix::Zero();
    Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (qa * dt * dt * dt / 3.0 + qp * dt);
    Q.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * (qa * dt * dt / 2.0);
    Q.block<3, 3>(3, 0) = Q.block<3, 3>(0, 3);
    Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (qa * dt);
    Q(6, 6) = qpsi * dt;
    Q.block<3, 3>(7, 7) = Eigen::Matrix3d::Identity() * (qb * dt);
    // Process noise only on the axes that are being estimated: an axis with no
    // prior is pinned at zero, and giving it noise would let it wander into the
    // position states it cannot be told apart from.
    const double qc = config_.camera_bias_noise_density * config_.camera_bias_noise_density;
    const Eigen::Vector3d prior = cameraBiasPrior();
    for (int i = 0; i < 3; ++i) {
      Q(10 + i, 10 + i) = prior(i) > 0.0 ? qc * dt : 0.0;
    }

    covariance_ = F * covariance_ * F.transpose() + Q;
    symmetrize();
  }

  // ---- Measurement models ----------------------------------------------------
  Result applyPose(const Event &event, bool is_replay) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, kStateDim);
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    const Eigen::VectorXd innovation = event.vector - state_.segment<3>(0);
    const Eigen::MatrixXd R =
        Eigen::Matrix3d::Identity() * config_.pose_sigma * config_.pose_sigma;
    return update(innovation, H, R, /*gate=*/true, is_replay, event.was_applied);
  }

  Result applyCorners(const Event &event, bool is_replay) {
    Eigen::VectorXd innovation;
    Eigen::MatrixXd H;
    Eigen::MatrixXd attitude_jacobian;
    Eigen::MatrixXd extrinsic_jacobian;
    std::vector<Eigen::Vector2d> projected;
    if (!cornerModel(state_, *event.corners, innovation, H, projected, &attitude_jacobian,
                     &extrinsic_jacobian)) {
      // The estimate says the tag is behind the camera, and yet a detector just
      // read it. That is not a missing measurement, it is a statement that the
      // ESTIMATE is wrong -- so it has to drive the recovery timer like any other
      // rejection. Returning quietly here was a second one-way door, hidden behind
      // the one that was already fixed: a filter whose position had run away
      // stopped even being offered measurements, and sat there dead-reckoning for
      // the rest of the flight while its rejection count stood still.
      if (!is_replay) {
        ++rejected_count_;
        ++consecutive_rejects_;
      }
      return Result::kInvalid;
    }
    const int rows = static_cast<int>(innovation.size());
    Eigen::MatrixXd R = Eigen::MatrixXd::Identity(rows, rows) * config_.pixel_sigma *
                        config_.pixel_sigma;
    // Attitude uncertainty, projected into pixels. Off-diagonal by construction:
    // one attitude error moves every corner of the frame the same way.
    R += attitude_jacobian * (config_.attitude_sigma * config_.attitude_sigma) *
         attitude_jacobian.transpose();

    // Camera mounting uncertainty, likewise common to the whole frame -- but only
    // while nothing is estimating it. Once b_cam is a state the uncertainty lives
    // in P, and leaving the full term here as well would count one unknown
    // constant twice and keep the tag discounted exactly where it was already
    // being discounted. What stays behind is the part three translation states
    // cannot represent, chiefly boresight rotation.
    // Per axis: an axis being estimated keeps only the residual here, because its
    // uncertainty now lives in P and counting it twice would keep the tag
    // discounted exactly where it was already being discounted. An axis that is
    // NOT estimated keeps the full term, because nothing else is carrying it.
    const Eigen::Vector3d prior = cameraBiasPrior();
    Eigen::Vector3d extrinsic_sigma;
    for (int i = 0; i < 3; ++i) {
      extrinsic_sigma(i) = prior(i) > 0.0 ? config_.extrinsic_residual_sigma
                                          : config_.extrinsic_sigma;
    }
    R += extrinsic_jacobian * extrinsic_sigma.array().square().matrix().asDiagonal() *
         extrinsic_jacobian.transpose();

    // Unknown pipeline delay, projected the same way: an unmodelled delay dt puts
    // the vehicle v*dt from where the measurement thinks it was, so the pixel
    // error is H_p v dt -- one direction, one rank.
    const Eigen::VectorXd lag = H.leftCols<3>() * state_.segment<3>(3);
    R += (config_.latency_sigma * config_.latency_sigma) * (lag * lag.transpose());
    return update(innovation, H, R, /*gate=*/true, is_replay, event.was_applied,
                  /*measures_pixels=*/true);
  }

  // The navigation velocity measures the true relative velocity PLUS its own bias.
  // Writing that down is the whole point of b_v: the filter can now tell a vehicle
  // that is moving from a navigation solution that thinks it is, and it carries the
  // difference forward as uncertainty rather than as confidence.
  Result applyVelocity(const Event &event) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, kStateDim);
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(0, 7) = Eigen::Matrix3d::Identity();
    const Eigen::VectorXd innovation =
        event.vector - state_.segment<3>(3) - state_.segment<3>(7);
    const Eigen::MatrixXd R =
        Eigen::Matrix3d::Identity() * config_.velocity_sigma * config_.velocity_sigma;
    return update(innovation, H, R, /*gate=*/false, /*is_replay=*/false, /*was_applied=*/true);
  }

  Result applyRangeAltitude(const Event &event) {
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, kStateDim);
    H(0, 2) = 1.0;
    Eigen::VectorXd innovation(1);
    innovation(0) = event.scalar - state_(2);
    Eigen::MatrixXd R(1, 1);
    const double sigma = event.scalar_low > 0.0 ? event.scalar_low : config_.range_sigma;
    R(0, 0) = sigma * sigma;
    return update(innovation, H, R, /*gate=*/false, /*is_replay=*/false, /*was_applied=*/true);
  }

  Result applyAltitudeBounds(Event &event, bool is_replay) {
    // Only the violated side is applied, and a replay repeats whichever side was
    // applied the first time so the rewind stays exact.
    int side = 0;
    if (is_replay) {
      side = event.applied_side;
    } else if (state_(2) > event.scalar) {
      side = 1;
    } else if (state_(2) < event.scalar_low) {
      side = -1;
    }
    event.applied_side = side;
    if (side == 0) {
      return Result::kRejected;
    }
    const double bound = side > 0 ? event.scalar : event.scalar_low;
    const double variance = covariance_(2, 2);
    if (!(variance > 0.0)) {
      return Result::kInvalid;
    }
    // x <- x + P H' (H P H')^-1 (bound - Hx), with the covariance untouched.
    state_ += covariance_.col(2) * ((bound - state_(2)) / variance);
    state_(6) = wrapToPi(state_(6));
    return Result::kApplied;
  }

  // Projection of every corner in the observation, and the Jacobian of that
  // projection with respect to the state.
  //
  //   d_C = (R_WB R_BC)' * ( Rz(psi) c  -  (p + R_WB r_cb) )
  //   u   = fx d_x / d_z + cx,   v = fy d_y / d_z + cy
  //
  // so the position block is -J_pix M' and the yaw column is J_pix M' dRz(psi) c.
  // Note the range scaling nobody had to invent: J_pix carries 1/d_z, so the same
  // pixel sigma becomes a larger position uncertainty at altitude, automatically.
  bool cornerModel(const StateVector &x, const CornerObservation &obs,
                   Eigen::VectorXd &innovation, Eigen::MatrixXd &H,
                   std::vector<Eigen::Vector2d> &projected,
                   Eigen::MatrixXd *attitude_jacobian = nullptr,
                   Eigen::MatrixXd *extrinsic_jacobian = nullptr) const {
    const std::size_t n = obs.pixels.size();
    const double psi = x(6);
    const double c_psi = std::cos(psi);
    const double s_psi = std::sin(psi);

    Eigen::Matrix3d Rz;
    Rz << c_psi, -s_psi, 0.0,
          s_psi,  c_psi, 0.0,
            0.0,    0.0, 1.0;
    Eigen::Matrix3d dRz;
    dRz << -s_psi, -c_psi, 0.0,
            c_psi, -s_psi, 0.0,
              0.0,    0.0, 0.0;

    // Camera optical axes expressed in world: columns of M. Its transpose takes a
    // world vector into the optical frame.
    const Eigen::Matrix3d M = obs.R_world_body * obs.R_body_camera;
    const Eigen::Matrix3d Mt = M.transpose();
    // The mounting offset is the nominal one plus whatever b_cam has learned. With
    // the state off, b_cam stays at zero and this is the nominal offset exactly.
    const Eigen::Vector3d r_camera_body = obs.r_camera_body + x.segment<3>(10);
    const Eigen::Vector3d camera_position = x.segment<3>(0) + obs.R_world_body * r_camera_body;

    innovation.resize(static_cast<int>(2 * n));
    H = Eigen::MatrixXd::Zero(static_cast<int>(2 * n), kStateDim);
    projected.resize(n);
    if (attitude_jacobian != nullptr) {
      *attitude_jacobian = Eigen::MatrixXd::Zero(static_cast<int>(2 * n), 3);
    }
    if (extrinsic_jacobian != nullptr) {
      *extrinsic_jacobian = Eigen::MatrixXd::Zero(static_cast<int>(2 * n), 3);
    }

    for (std::size_t i = 0; i < n; ++i) {
      const Eigen::Vector3d corner_world = Rz * obs.points_platform[i];
      const Eigen::Vector3d d_camera = Mt * (corner_world - camera_position);
      if (!(d_camera(2) > config_.min_depth)) {
        return false;  // behind the camera or implausibly close: not a measurement
      }
      const double inv_z = 1.0 / d_camera(2);
      const Eigen::Vector2d pixel(obs.camera.fx * d_camera(0) * inv_z + obs.camera.cx,
                                  obs.camera.fy * d_camera(1) * inv_z + obs.camera.cy);
      projected[i] = pixel;

      Eigen::Matrix<double, 2, 3> J_pix;
      J_pix << obs.camera.fx * inv_z, 0.0, -obs.camera.fx * d_camera(0) * inv_z * inv_z,
               0.0, obs.camera.fy * inv_z, -obs.camera.fy * d_camera(1) * inv_z * inv_z;

      const int row = static_cast<int>(2 * i);
      innovation.segment<2>(row) = obs.pixels[i] - pixel;
      H.block<2, 3>(row, 0) = -J_pix * Mt;
      H.block<2, 1>(row, 6) = J_pix * Mt * (dRz * obs.points_platform[i]);
      // Same derivative the extrinsic term in R is built from, now used as a
      // Jacobian instead: the offset is fixed in body axes, so it reaches the
      // optical frame through R_bc alone. Axes with no prior have zero covariance,
      // so their columns cannot move the state whatever this says.
      H.block<2, 3>(row, 10) = -J_pix * obs.R_body_camera.transpose();

      if (attitude_jacobian != nullptr) {
        // Derivative with respect to a body-frame attitude perturbation
        // R_wb <- R_wb exp(delta^). The camera offset drops out of it, because it
        // is fixed in the body frame and turns with it:
        //   d_C = R_bc' [ exp(-delta^) u - r_cb ],  u = R_wb' (corner - p)
        //   dd_C/ddelta = R_bc' skew(u)
        const Eigen::Vector3d u =
            obs.R_world_body.transpose() * (corner_world - x.segment<3>(0));
        (void)r_camera_body;
        attitude_jacobian->block<2, 3>(row, 0) =
            J_pix * obs.R_body_camera.transpose() * skew(u);
      }
      if (extrinsic_jacobian != nullptr) {
        // d_C depends on the mounting offset as -R_bc' r_cb, so this is the same
        // block the position states get, expressed in body axes.
        extrinsic_jacobian->block<2, 3>(row, 0) = -J_pix * obs.R_body_camera.transpose();
      }
    }
    return true;
  }

  static Eigen::Matrix3d skew(const Eigen::Vector3d &v) {
    Eigen::Matrix3d m;
    m << 0.0, -v(2), v(1),
         v(2), 0.0, -v(0),
         -v(1), v(0), 0.0;
    return m;
  }

  // ---- The update itself -----------------------------------------------------
  Result update(const Eigen::VectorXd &innovation, const Eigen::MatrixXd &H,
                const Eigen::MatrixXd &R, bool gate, bool is_replay, bool was_applied,
                bool measures_pixels = false) {
    // Nothing has been accepted for a while: the estimate, not the world, is the
    // thing most likely to be wrong. Widen the covariance BEFORE forming the gain
    // so the correction that follows is taken with an honest uncertainty, and let
    // this measurement through. Without it the gate is a one-way door -- rejection
    // starves the estimate, the estimate drifts further, and the next innovation
    // is larger still. That spiral was measured: NIS 27, then 157, then 2085.
    const bool locked_out = gate && !is_replay && bootstrap_remaining_ == 0 &&
                            (time_ - last_accepted_time_) > config_.reject_recovery_seconds;
    if (locked_out) {
      covariance_ *= config_.reject_inflation_factor;
    }

    const Eigen::MatrixXd PHt = covariance_ * H.transpose();
    const Eigen::MatrixXd S = H * PHt + R;
    const Eigen::LDLT<Eigen::MatrixXd> ldlt(S);
    if (ldlt.info() != Eigen::Success) {
      return Result::kInvalid;
    }

    const int dof = static_cast<int>(innovation.size());
    const double nis = innovation.dot(ldlt.solve(innovation));
    const double threshold = chiSquaredThreshold(dof);

    if (gate) {
      if (!is_replay) {
        last_nis_ = nis;
        last_nis_dof_ = dof;
        last_gate_threshold_ = threshold;
        if (measures_pixels) {
          last_residual_rms_ = std::sqrt(innovation.squaredNorm() / dof);
          last_residual_predicted_rms_ = std::sqrt(S.diagonal().mean());
        }
      }
      const bool bootstrap = bootstrap_remaining_ > 0;
      const bool accept = is_replay ? was_applied : (bootstrap || nis <= threshold || locked_out);
      if (!accept) {
        if (!is_replay) {
          ++rejected_count_;
          ++consecutive_rejects_;
        }
        return Result::kRejected;
      }
      if (!is_replay) {
        ++applied_count_;
        consecutive_rejects_ = 0;
        last_accepted_time_ = time_;
        if (bootstrap_remaining_ > 0) {
          --bootstrap_remaining_;
        } else if (!locked_out) {
          // A measurement forced through the lockout is not evidence about the
          // covariance, so it does not go into the statistics that judge it.
          nis_normalised_sum_ += nis / static_cast<double>(dof);
          ++nis_samples_;
        }
      }
    }

    const Eigen::MatrixXd K = ldlt.solve(PHt.transpose()).transpose();
    state_ += K * innovation;
    state_(6) = wrapToPi(state_(6));
    // Bound the velocity bias. It is the state most able to wreck the others: it
    // enters the propagation, so a bias soaked up from a large position innovation
    // walks the estimate away at that speed, and the estimate walking away is what
    // put the tag outside the predicted image in the first place. Measured before
    // this bound: a filter started 1 m out attributed the correction to b_v,
    // reached 0.67 m/s of it, and then ran away at exactly that rate. Nothing
    // EKF2's position derivative does justifies more than this.
    state_.segment<3>(7) = state_.segment<3>(7).cwiseMax(-config_.velocity_bias_max)
                               .cwiseMin(config_.velocity_bias_max);
    state_.segment<3>(10) = state_.segment<3>(10).cwiseMax(-config_.camera_bias_max)
                                .cwiseMin(config_.camera_bias_max);

    // Joseph form: it costs one extra product and keeps the covariance symmetric
    // and positive definite through thousands of updates, which matters here
    // because the NIS statistics are the acceptance criterion.
    const StateMatrix IKH = StateMatrix::Identity() - K * H;
    covariance_ = IKH * covariance_ * IKH.transpose() + K * R * K.transpose();
    symmetrize();
    return Result::kApplied;
  }

  // Prior standard deviation per camera-bias axis; zero means "not estimated",
  // which pins the axis at nominal through both the covariance and Q.
  Eigen::Vector3d cameraBiasPrior() const {
    return config_.estimate_camera_bias ? config_.initial_camera_bias_sigma
                                        : Eigen::Vector3d::Zero();
  }

  void symmetrize() { covariance_ = 0.5 * (covariance_ + covariance_.transpose()).eval(); }

  // Wilson-Hilferty: the chi-squared quantile through the normal one, good to a
  // fraction of a percent for the degrees of freedom used here (1-16) and worth
  // far more than a lookup table nobody can read.
  double chiSquaredThreshold(int dof) const {
    const double k = static_cast<double>(dof);
    const double t = 1.0 - 2.0 / (9.0 * k) + gate_z_ * std::sqrt(2.0 / (9.0 * k));
    return k * t * t * t;
  }

  // Inverse normal CDF by Newton on erfc. Evaluated once per configuration.
  static double normalQuantile(double probability) {
    const double p = std::clamp(probability, 1e-9, 1.0 - 1e-9);
    double z = 0.0;
    for (int i = 0; i < 60; ++i) {
      const double cdf = 0.5 * std::erfc(-z / std::sqrt(2.0));
      const double pdf = std::exp(-0.5 * z * z) / std::sqrt(2.0 * M_PI);
      if (pdf < 1e-300) {
        break;
      }
      const double step = (cdf - p) / pdf;
      z -= step;
      if (std::abs(step) < 1e-12) {
        break;
      }
    }
    return z;
  }

  static double wrapToPi(double angle) { return std::remainder(angle, 2.0 * M_PI); }

  RelativeStateFilterConfig config_;
  double gate_z_ = 3.0902;

  StateVector state_ = StateVector::Zero();
  StateMatrix covariance_ = StateMatrix::Identity();
  double time_ = 0.0;
  bool initialized_ = false;

  std::deque<Event> history_;

  double last_residual_rms_ = 0.0;
  double last_residual_predicted_rms_ = 0.0;
  double last_nis_ = 0.0;
  int last_nis_dof_ = 0;
  double last_gate_threshold_ = 0.0;
  double nis_normalised_sum_ = 0.0;
  long nis_samples_ = 0;
  long applied_count_ = 0;
  long rejected_count_ = 0;
  int consecutive_rejects_ = 0;
  int bootstrap_remaining_ = 0;
  double last_accepted_time_ = 0.0;
};

}  // namespace landing

#endif  // RELATIVE_STATE_FILTER_H
