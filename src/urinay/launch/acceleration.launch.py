from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def generate_launch_description():
    config = get_package_share_directory('urinay') + '/config'
    parameters = {}
    for name in ('common.yml', 'acceleration.yml'):
        with open(os.path.join(config, name), 'r') as stream:
            parameters.update(yaml.safe_load(stream) or {})
    return LaunchDescription([
        Node(package='urinay', executable='urinay_exec', name='urinay',
             namespace='/AS/P', output='screen', parameters=[parameters])
    ])
