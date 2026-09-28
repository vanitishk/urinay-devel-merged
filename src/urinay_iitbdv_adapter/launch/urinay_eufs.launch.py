import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    urinay_config_dir = os.path.join(
        get_package_share_directory('urinay'),
        'config',
    )

    parameters = {}
    for filename in ('common.yml', 'autocross.yml'):
        config_path = os.path.join(urinay_config_dir, filename)
        with open(config_path, 'r', encoding='utf-8') as config_file:
            parameters.update(yaml.safe_load(config_file) or {})

    adapter = Node(
        package='urinay_iitbdv_adapter',
        executable='cone_pose_adapter',
        name='urinay_iitbdv_adapter',
        output='screen',
        parameters=[{
            'input_cones_topic': '/perception/cones',
            'input_pose_topic': '/ground_truth/odom',
            'output_cones_topic': '/urinay/cones_cartesian',
            'cone_match_distance': 0.4,
        }],
    )

    urinay = Node(
        package='urinay',
        executable='urinay_exec',
        name='urinay',
        output='screen',
        parameters=[parameters],
    )

    return LaunchDescription([adapter, urinay])