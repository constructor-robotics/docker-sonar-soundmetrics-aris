import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    pkg_share = get_package_share_directory('soundmetrics_aris_drivers')

    ns_arg = DeclareLaunchArgument(
        'ns',
        default_value='soundmetrics_aris3000',
        description='Namespace for the sonar nodes'
    )

    yaml_path = os.path.join(pkg_share, 'config', 'soundmetrics_aris3000__standard.yaml')

    with open(yaml_path, 'r') as f:
        yaml_params = yaml.safe_load(f)
    enable_cartesian = str(
        yaml_params.get('/**', {}).get('ros__parameters', {}).get('enable_cartesian', True)
    ).lower()

    enable_cartesian_arg = DeclareLaunchArgument(
        'enable_cartesian',
        default_value=enable_cartesian,
        description='Enable the polar-to-cartesian converter node (default from YAML)'
    )

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
        parameters=[yaml_path],
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
