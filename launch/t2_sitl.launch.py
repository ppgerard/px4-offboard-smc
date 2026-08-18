import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, NotEqualsSubstitution
from launch_ros.actions import Node


def generate_launch_description():
   config_1 = os.path.join(
      get_package_share_directory('px4_offboard_lowlevel'),
      'config', 'uav_parameters',
      't2_param.yaml'
      )

   config_2 = os.path.join(
      get_package_share_directory('px4_offboard_lowlevel'),
      'config', 'sitl',
      'sitl_params.yaml'
      )

   config_3 = os.path.join(
      get_package_share_directory('px4_offboard_lowlevel'),
      'config', 'controller',
      'initial_gains_t2.yaml'
      )

   trajectory_arg = DeclareLaunchArgument(
      'trajectory',
      default_value='none',
      description='Trajectory publisher to launch alongside the controller: '
                   'none, circle, landing, px4_landing, tuning'
   )
   trajectory = LaunchConfiguration('trajectory')

   controller_type_arg = DeclareLaunchArgument(
      'controller_type',
      default_value='smc',
      description='Control law used by offboard_controller_node: smc or stsmc'
   )
   controller_type = LaunchConfiguration('controller_type')

   estimator_arg = DeclareLaunchArgument(
      'estimator',
      default_value='complementary',
      description='Estimator the landing guidance steers on: complementary '
                   '(fixed-gain filter), ekf (relative-state EKF on tag corner '
                   'pixels) or ekf_pose (the same EKF fed the TF pose, for bring-up). '
                   'Both estimators run and are published either way.'
   )
   estimator = LaunchConfiguration('estimator')

   return LaunchDescription([
      trajectory_arg,
      controller_type_arg,
      estimator_arg,
      # Controller. Not launched for 'px4_landing', which bypasses it and
      # sends setpoints straight to PX4 (running both would fight for control).
      Node(
         package='px4_offboard_lowlevel',
         executable='offboard_controller_node',
         name='offboard_controller',
         parameters=[config_1, config_2, config_3, {'controller_type': controller_type}],
         condition=IfCondition(NotEqualsSubstitution(trajectory, 'px4_landing')),
      ),
      Node(
         package='px4_offboard_lowlevel',
         executable='circle_trajectory_node',
         name='circle_trajectory_publisher',
         condition=IfCondition(EqualsSubstitution(trajectory, 'circle')),
      ),
      Node(
         package='px4_offboard_lowlevel',
         executable='landing_trajectory_node',
         name='landing_trajectory_publisher',
         parameters=[config_1, {'landing_parameters.estimator': estimator}],
         condition=IfCondition(EqualsSubstitution(trajectory, 'landing')),
      ),
      Node(
         package='px4_offboard_lowlevel',
         executable='px4_offboard_landing_node',
         name='px4_offboard_trajectory_publisher',
         parameters=[config_1, {'landing_parameters.estimator': estimator}],
         condition=IfCondition(EqualsSubstitution(trajectory, 'px4_landing')),
      ),
      Node(
         package='px4_offboard_lowlevel',
         executable='steps_publisher_node',
         name='steps_publisher',
         condition=IfCondition(EqualsSubstitution(trajectory, 'tuning')),
      ),
   ])