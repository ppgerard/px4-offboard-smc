"""Indoor / no-GNSS hover test, flyable with the SMC/STSMC or with PX4's own controller.

Three nodes and nothing else:

  offboard_controller_node  the control law under test
  tag_ev_bridge_node        AprilTag -> /fmu/in/vehicle_visual_odometry, so EKF2
                            has a horizontal aiding source at all
  indoor_hover_node         a bounded hold-and-step profile over the tag

The landing stack is deliberately absent. This rig answers "is the control law
stable on this airframe"; the landing rig answers "does the whole guidance stack
land", and running both at once is how you end up unable to say which half failed.

  mode:=sitl   simulator topics and the gz actuator scaling (px4InverseSITL)
  mode:=hw     the real vehicle: exp topic names and the PWM curve (px4Inverse)

EKF2 MUST be told to use the bridge, or this flies on the fake-position anchor
with no warning anywhere in the stack. See tools/indoor_ev_test.sh for the
parameter block, and set the same values on the vehicle:

  EKF2_EV_CTRL 1      bit 0, horizontal position only -- height stays on
                      baro/rangefinder, which do not need GNSS
  EKF2_GPS_CTRL 0     no GNSS indoors
  EKF2_HGT_REF 2      rangefinder as the height reference, baro as an aid. Baro
                      as the reference was measured drifting 0.6 m over a 60 s
                      hover and flew the aircraft into the ground while the
                      estimate read 0.70 m.
  EKF2_EV_DELAY       the measured camera+detector latency, in ms
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('px4_offboard_lowlevel')

    sitl_uav = os.path.join(share, 'config', 'uav_parameters', 't2_param.yaml')
    hw_uav = os.path.join(share, 'config', 'exp', 't2_hw_param.yaml')
    sitl_topics = os.path.join(share, 'config', 'sitl', 'sitl_params.yaml')
    hw_topics = os.path.join(share, 'config', 'exp', 'exp_params.yaml')
    gains = os.path.join(share, 'config', 'controller', 'initial_gains_t2.yaml')

    args = [
        DeclareLaunchArgument('mode', default_value='sitl',
                              description='sitl or hw'),
        DeclareLaunchArgument('controller_type', default_value='stsmc',
                              description="smc, stsmc, or 'px4' to fly the identical "
                                          "profile with PX4's own position controller. "
                                          "Fly px4 FIRST on any new airframe: it separates "
                                          "'is the estimate good enough' from 'is the "
                                          "control law good enough'."),
        DeclareLaunchArgument('enable_steps', default_value='false',
                              description='Run the step sequence after the hold. '
                                          'Leave false for a first flight.'),
        DeclareLaunchArgument('altitude', default_value='1.2',
                              description='Hold altitude above the tag [m]'),
        DeclareLaunchArgument('ev_bridge', default_value='true',
                              description='Publish external vision from the tag. '
                                          'false only if another source (mocap, GNSS) '
                                          'is already aiding EKF2.'),
    ]

    mode = LaunchConfiguration('mode')
    ctrl = LaunchConfiguration('controller_type')
    # PX4's own controller and offboard_controller_node are mutually exclusive:
    # one publishes OffboardControlMode with position=true, the other with
    # direct_actuator=true, and PX4 acts on whichever arrived last.
    use_px4 = PythonExpression(["'", ctrl, "' == 'px4'"])
    is_sitl = PythonExpression(["'", mode, "' == 'sitl' and '", ctrl, "' != 'px4'"])
    is_hw = PythonExpression(["'", mode, "' != 'sitl' and '", ctrl, "' != 'px4'"])

    def controller(config_uav, config_topics, condition):
        return Node(
            package='px4_offboard_lowlevel',
            executable='offboard_controller_node',
            name='offboard_controller',
            parameters=[config_uav, config_topics, gains,
                        {'controller_type': LaunchConfiguration('controller_type')}],
            condition=IfCondition(condition),
            output='screen',
        )

    def bridge(config_uav, condition):
        return Node(
            package='px4_offboard_lowlevel',
            executable='tag_ev_bridge_node',
            name='tag_ev_bridge',
            parameters=[config_uav],
            condition=IfCondition(condition),
            output='screen',
        )

    return LaunchDescription(args + [
        controller(sitl_uav, sitl_topics, is_sitl),
        controller(hw_uav, hw_topics, is_hw),
        bridge(sitl_uav, PythonExpression(
            ["'", LaunchConfiguration('ev_bridge'), "' == 'true' and '", mode, "' == 'sitl'"])),
        bridge(hw_uav, PythonExpression(
            ["'", LaunchConfiguration('ev_bridge'), "' == 'true' and '", mode, "' != 'sitl'"])),
        Node(
            package='px4_offboard_lowlevel',
            executable='indoor_hover_node',
            name='indoor_hover',
            parameters=[{
                'hover.altitude': ParameterValue(LaunchConfiguration('altitude'), value_type=float),
                'hover.enable_steps': ParameterValue(LaunchConfiguration('enable_steps'), value_type=bool),
                'hover.use_px4_controller': ParameterValue(use_px4, value_type=bool),
            }],
            output='screen',
        ),
    ])
