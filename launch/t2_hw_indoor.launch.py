"""Everything the real T2 needs indoors, in one launch.

    ros2 launch px4_offboard_lowlevel t2_hw_indoor.launch.py

Brings up, in this order:

  camera_36h11.launch.yml   camera_ros -> image_proc rectify -> apriltag detector
                            (from the apriltag_ros package), publishing
                            /apriltag/detections and the `platform` TF
  tag_ev_bridge_node        platform TF -> /fmu/in/vehicle_visual_odometry, so
                            EKF2 has a horizontal aiding source at all
  offboard_controller_node  the SMC/STSMC   (skipped when controller_type:=px4)
  indoor_hover_node         the bounded hold-and-step profile

It does NOT start the uXRCE-DDS agent -- that belongs to the link, not to this
stack, and it has to be up before anything here is useful:

    MicroXRCEAgent serial --dev /dev/ttyAMA0 -b 921600     # adjust to your wiring

WHAT TO FLY, IN ORDER. Each step is a superset of the one before, and each one
can fail in a way the next cannot diagnose:

  1. camera_only:=true, everything else off. Verify /landing/tag_in_body has the
     right SIGNS before anything is armed. See the table in tag_ev_bridge_node.
  2. controller_type:=px4, enable_steps:=false. PX4's own position controller
     holds station on the tag. If this cannot hold, the estimate is the problem
     and nothing measured after it means anything.
  3. controller_type:=px4, enable_steps:=true.
  4. controller_type:=stsmc, enable_steps:=false, then true.

TAKE OFF MANUALLY, ON THE RC, IN ALTITUDE MODE, AND HAND OVER AT ALTITUDE.
The camera sits ~5 mm above the tag when the aircraft is on its gear, so nothing
decodes on the pad: the bridge publishes ~0.1 Hz on the ground and ~15 Hz once
airborne. There is no horizontal aiding during the climb and there is no way to
give it any -- the manual takeoff exists to cover exactly that window. Confirm
xy_valid before you hand over.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('px4_offboard_lowlevel')
    hw_uav = os.path.join(share, 'config', 'exp', 't2_hw_param.yaml')
    hw_topics = os.path.join(share, 'config', 'exp', 'exp_params.yaml')
    gains = os.path.join(share, 'config', 'controller', 'initial_gains_t2.yaml')

    camera_launch = os.path.join(
        get_package_share_directory('apriltag_ros'), 'launch', 'camera_36h11.launch.yml')

    args = [
        DeclareLaunchArgument('controller_type', default_value='px4',
                              description="px4 (PX4's own position controller -- fly this "
                                          "FIRST), or smc / stsmc."),
        DeclareLaunchArgument('enable_steps', default_value='false',
                              description='Run the x/y/z step sequence after the hold. '
                                          'Leave false for a first flight.'),
        DeclareLaunchArgument('altitude', default_value='1.2',
                              description='Hold altitude above the tag [m]'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Start the camera and detector. false if they '
                                          'are already running in another terminal.'),
        DeclareLaunchArgument('camera_only', default_value='false',
                              description='Camera, detector and EV bridge ONLY -- no '
                                          'controller, no setpoints, nothing that can '
                                          'command the aircraft. This is the extrinsic '
                                          'check of step 1.'),
        DeclareLaunchArgument('device', default_value='0',
                              description='camera_ros device index'),
    ]

    ctrl = LaunchConfiguration('controller_type')
    camera_only = LaunchConfiguration('camera_only')

    # camera_only wins over everything: it must not be possible to ask for the
    # extrinsic check and get a node that publishes setpoints as well.
    fly = PythonExpression(["'", camera_only, "' != 'true'"])
    run_smc = PythonExpression(
        ["'", camera_only, "' != 'true' and '", ctrl, "' != 'px4'"])

    return LaunchDescription(args + [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(camera_launch),
            launch_arguments={'device': LaunchConfiguration('device')}.items(),
            condition=IfCondition(LaunchConfiguration('camera')),
        ),
        Node(
            package='px4_offboard_lowlevel',
            executable='tag_ev_bridge_node',
            name='tag_ev_bridge',
            parameters=[hw_uav],
            output='screen',
        ),
        Node(
            package='px4_offboard_lowlevel',
            executable='offboard_controller_node',
            name='offboard_controller',
            parameters=[hw_uav, hw_topics, gains, {'controller_type': ctrl}],
            condition=IfCondition(run_smc),
            output='screen',
        ),
        Node(
            package='px4_offboard_lowlevel',
            executable='indoor_hover_node',
            name='indoor_hover',
            parameters=[hw_uav, hw_topics, {
                'hover.altitude': ParameterValue(
                    LaunchConfiguration('altitude'), value_type=float),
                'hover.enable_steps': ParameterValue(
                    LaunchConfiguration('enable_steps'), value_type=bool),
                'hover.use_px4_controller': ParameterValue(
                    PythonExpression(["'", ctrl, "' == 'px4'"]), value_type=bool),
            }],
            condition=IfCondition(fly),
            output='screen',
        ),
    ])
