"""Everything the real T2 needs, in one launch. Indoors or outdoors.

    ros2 launch px4_offboard_lowlevel t2_hw.launch.py

Brings up, in this order:

  camera_36h11.launch.yml   camera_ros -> image_proc rectify -> apriltag detector
                            (from the apriltag_ros package), publishing
                            /apriltag/detections and the `platform` TF
  tag_ev_bridge_node        platform TF -> /fmu/in/vehicle_visual_odometry, so
                            EKF2 has a horizontal aiding source at all
  offboard_controller_node  the SMC/STSMC   (skipped when controller_type:=px4)
  the trajectory source     indoor_hover_node (hold + steps), landing_trajectory_node
                            (the real landing) or px4_offboard_landing_node (the
                            same landing flown by PX4's own position controller)

OUTDOORS, with GNSS: pass ev_bridge:=false and restore EKF2_GPS_CTRL 7,
EKF2_EV_CTRL 0, EKF2_HGT_REF 1, NAV_RCL_ACT 2. That is the configuration every
result in CLAUDE.md was measured with.

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
        DeclareLaunchArgument('trajectory', default_value='hover',
                              description='hover (hold + optional steps), landing (the '
                                          'SMC/STSMC landing) or px4_landing (the same '
                                          'landing guidance flown by PX4 itself).'),
        DeclareLaunchArgument('estimator', default_value='ekf',
                              description='Landing estimator that STEERS: ekf, '
                                          'complementary or ekf_pose. Both always run.'),
        DeclareLaunchArgument('device', default_value='0',
                              description='camera_ros device index'),
        DeclareLaunchArgument('ev_bridge', default_value='true',
                              description='Feed the tag to EKF2 as external vision. '
                                          'Set FALSE outdoors, where GNSS is the position '
                                          'source and EKF2_EV_CTRL is 0 -- the bridge would '
                                          'just be publishing into a topic nothing fuses.'),
    ]

    ctrl = LaunchConfiguration('controller_type')
    camera_only = LaunchConfiguration('camera_only')

    # camera_only wins over everything: it must not be possible to ask for the
    # extrinsic check and get a node that publishes setpoints as well.
    traj = LaunchConfiguration('trajectory')
    armed = PythonExpression(["'", camera_only, "' != 'true'"])

    def when(expr):
        return IfCondition(PythonExpression(["'", camera_only, "' != 'true' and (", expr, ")"]))

    # The controller is skipped for px4_landing (which sends TrajectorySetpoints
    # straight to PX4) and for controller_type:=px4 -- in both cases PX4 flies, and
    # running ours too would have two sources publishing OffboardControlMode.
    run_smc = when(PythonExpression(
        ["'", traj, "' != 'px4_landing' and '", ctrl, "' != 'px4'"]))
    run_hover = when(PythonExpression(["'", traj, "' == 'hover'"]))
    run_landing = when(PythonExpression(["'", traj, "' == 'landing'"]))
    run_px4_landing = when(PythonExpression(["'", traj, "' == 'px4_landing'"]))

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
            condition=IfCondition(LaunchConfiguration('ev_bridge')),
            output='screen',
        ),
        Node(
            package='px4_offboard_lowlevel',
            executable='offboard_controller_node',
            name='offboard_controller',
            parameters=[hw_uav, hw_topics, gains, {'controller_type': ctrl}],
            condition=run_smc,
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
            condition=run_hover,
            output='screen',
        ),
        Node(
            package='px4_offboard_lowlevel',
            executable='landing_trajectory_node',
            name='landing_trajectory_publisher',
            parameters=[hw_uav, hw_topics,
                        {'landing_parameters.estimator': LaunchConfiguration('estimator')}],
            condition=run_landing,
            output='screen',
        ),
        Node(
            package='px4_offboard_lowlevel',
            executable='px4_offboard_landing_node',
            name='px4_offboard_trajectory_publisher',
            parameters=[hw_uav, hw_topics,
                        {'landing_parameters.estimator': LaunchConfiguration('estimator')}],
            condition=run_px4_landing,
            output='screen',
        ),
    ])
