#ifndef PX4_OFFBOARD_LOWLEVEL_CONTROL_CONFIG_H
#define PX4_OFFBOARD_LOWLEVEL_CONTROL_CONFIG_H

namespace px4_offboard {

// Period of the control loop and of the trajectory publishers, in seconds.
//
// This is the single source of truth for the 100 Hz rate. It sets the wall
// timers of the controller and of every trajectory publisher, and it is the dt
// used for the numerical derivatives inside the control laws and the trajectory
// generators. Change it here and every one of those follows; there is no second
// copy to keep in sync.
inline constexpr double kControlPeriodSeconds = 0.01;

}  // namespace px4_offboard

#endif  // PX4_OFFBOARD_LOWLEVEL_CONTROL_CONFIG_H
