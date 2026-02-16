import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_share = get_package_share_directory('soundmetrics_aris_drivers')

    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='soundmetrics_aris3000',
        description='Namespace for the sonar nodes'
    )

    enable_cartesian_arg = DeclareLaunchArgument(
        'enable_cartesian',
        default_value='true',
        description='Enable the polar-to-cartesian converter node'
    )

    yaml_path = os.path.join(pkg_share, 'config', 'soundmetrics_aris3000__standard.yaml')

    sonar_node = Node(
        package='soundmetrics_aris_drivers',
        executable='soundmetrics_aris3000',
        name='soundmetrics_aris3000',
        output='screen',
        respawn=True,
        parameters=[yaml_path],
    )

    cartesian_node = Node(
        package='soundmetrics_aris_drivers',
        executable='polar_to_cartesian',
        name='polar_to_cartesian',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_cartesian')),
    )

    grouped = GroupAction([
        PushRosNamespace(LaunchConfiguration('ns')),
        sonar_node,
        cartesian_node,
    ])

    return LaunchDescription([
        ns_arg,
        enable_cartesian_arg,
        grouped,
    ])
