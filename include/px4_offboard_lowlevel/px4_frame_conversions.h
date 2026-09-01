#ifndef PX4_OFFBOARD_LOWLEVEL_PX4_FRAME_CONVERSIONS_H
#define PX4_OFFBOARD_LOWLEVEL_PX4_FRAME_CONVERSIONS_H

// Conversions between the PX4 (NED / FRD) and ROS (ENU / FLU) conventions.
// Shared by the controller node and the trajectory publishers so both sides
// interpret vehicle odometry identically.

#include <eigen3/Eigen/Eigen>
#include <px4_msgs/msg/vehicle_odometry.hpp>

namespace px4_frames {

// NED (X North, Y East, Z Down) <-> ENU (X East, Y North, Z Up)
inline Eigen::Vector3d rotateVectorFromToENU_NED(const Eigen::Vector3d &vec_in) {
    return Eigen::Vector3d(vec_in[1], vec_in[0], -vec_in[2]);
}

// FRD (X Forward, Y Right, Z Down) <-> FLU (X Forward, Y Left, Z Up)
inline Eigen::Vector3d rotateVectorFromToFRD_FLU(const Eigen::Vector3d &vec_in) {
    return Eigen::Vector3d(vec_in[0], -vec_in[1], -vec_in[2]);
}

// Transform an orientation between the ROS and PX4 conventions. Two steps:
//   1. aircraft-to-NED becomes aircraft-to-ENU (NED_to_ENU conversion)
//   2. aircraft-to-ENU becomes baselink-to-ENU (baselink_to_aircraft conversion)
// (or, in the other direction, baselink-to-ENU -> baselink-to-NED -> aircraft-to-NED).
inline Eigen::Quaterniond rotateQuaternionFromToENU_NED(const Eigen::Quaterniond &quat_in) {
    // Static quaternion rotating between the ENU and NED frames.
    static const Eigen::Quaterniond kNedToEnu(
        Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

    // Static quaternion rotating between the aircraft and base_link frames.
    static const Eigen::Quaterniond kAircraftToBaselink(
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));

    return (kNedToEnu * quat_in) * kAircraftToBaselink;
}

// Unpack a PX4 VehicleOdometry message into ROS-convention Eigen quantities.
//
// The velocity comes out in the WORLD (ENU) frame, and the message's own
// velocity_frame field decides how to get there. That field used to be ignored:
// the velocity was run through the NED->ENU conversion (correct, since EKF2
// publishes VELOCITY_FRAME_NED -- see EKF2::PublishOdometry) and then rotated by
// R_B_W a second time inside ControllerBase::setOdometry, as though it had been
// a body-frame quantity. The result is only right when R_B_W is the identity.
//
// SITL never showed it. The vehicle spawns aligned with the Gazebo world x-axis
// and the landing node commands an identity attitude, so the heading is ~0 for
// the whole flight and the spurious rotation IS the identity. On real hardware
// the EKF frame is north-aligned by the magnetometer and the aircraft sits at
// whatever heading the room gives it, so -m*Lambda*e_v -- the dominant
// horizontal authority at 3.73 N per m/s -- pushes rotated by the heading. At
// 90 degrees it is pure cross-coupling.
//
// The landing node already treated this output as world (drone_velocity_W_ =
// velocity), which is the inconsistency that gave it away.
inline void eigenOdometryFromPX4Msg(const px4_msgs::msg::VehicleOdometry::SharedPtr msg,
                                    Eigen::Vector3d &position_W, Eigen::Quaterniond &orientation_B_W,
                                    Eigen::Vector3d &velocity_W, Eigen::Vector3d &angular_velocity_B) {
    position_W = rotateVectorFromToENU_NED(
        Eigen::Vector3d(msg->position[0], msg->position[1], msg->position[2]));

    const Eigen::Quaterniond quaternion(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    orientation_B_W = rotateQuaternionFromToENU_NED(quaternion);

    const Eigen::Vector3d velocity_raw(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
        // The only genuinely body-fixed case, and the only one that needs the
        // attitude to reach world axes.
        orientation_B_W.normalize();
        velocity_W = orientation_B_W.toRotationMatrix() * rotateVectorFromToFRD_FLU(velocity_raw);
    } else {
        // NED, FRD and UNKNOWN are all navigation frames. EKF2 publishes NED.
        velocity_W = rotateVectorFromToENU_NED(velocity_raw);
    }

    angular_velocity_B = rotateVectorFromToFRD_FLU(
        Eigen::Vector3d(msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]));
}

}  // namespace px4_frames

#endif  // PX4_OFFBOARD_LOWLEVEL_PX4_FRAME_CONVERSIONS_H
