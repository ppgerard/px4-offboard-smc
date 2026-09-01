#ifndef PX4_OFFBOARD_LOWLEVEL_CAMERA_EXTRINSICS_H
#define PX4_OFFBOARD_LOWLEVEL_CAMERA_EXTRINSICS_H

#include <Eigen/Dense>
#include <cmath>

namespace px4_offboard {

// Rotation from the camera OPTICAL frame to body FLU.
//
// This constant lived inline in landing_trajectory_base.h and is now shared,
// because a SECOND consumer (the external-vision bridge) has to agree with it
// exactly. Two copies of a camera rotation is precisely how this project lost a
// week: R_b_cam_ was wrong by 180 deg about the boresight (rot_z(-pi/2) where
// the mounting needs +pi/2), and the synthetic tag tools mirrored the same
// constant, so the error cancelled everywhere except against a real camera.
// Anything that re-derives this instead of calling it can drift the same way.
//
// The default (+90 deg about z, then 180 deg about x) is the SITL mounting:
// camera_link at <pose>0 0 -0.10 0 1.5707 0</pose>, so Ry(90 deg) takes link x
// -> body -Z (boresight down), link y -> body +Y, link z -> body +X. The
// detector's translation is a solvePnP output and so is in the ROS optical
// convention (x right, y down, z forward), which relates to the link frame by
// optical_x = -link_y, optical_y = -link_z, optical_z = +link_x. Net: optical x
// -> body -Y, optical y -> body -X, optical z -> body -Z.
//
// On real hardware the mounting is whatever it is. The angles are parameters at
// both call sites for that reason -- but they must be given the SAME values, and
// only tools/landing_perception_test.sh (a real camera) can validate them. A
// synthetic tag shares this constant with the code it feeds, so the error
// cancels and the test passes regardless.
inline Eigen::Matrix3d bodyFromCameraOptical(double rot_z_deg = 90.0,
                                             double rot_x_deg = 180.0) {
    const double kDegToRad = M_PI / 180.0;
    const Eigen::AngleAxisd rot_z(rot_z_deg * kDegToRad, Eigen::Vector3d::UnitZ());
    const Eigen::AngleAxisd rot_x(rot_x_deg * kDegToRad, Eigen::Vector3d::UnitX());
    return (rot_x * rot_z).toRotationMatrix();
}

}  // namespace px4_offboard

#endif  // PX4_OFFBOARD_LOWLEVEL_CAMERA_EXTRINSICS_H
