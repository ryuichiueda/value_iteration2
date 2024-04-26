import launch, os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
      get_package_share_directory('value_iteration2'),
      'config',
      'params.yaml'
    )

    vi_node = Node(
            package='value_iteration2',
            namespace='value_iteration2',
            executable='vi_node',
            name='vi_node',
            parameters=[config],
        )

    return launch.LaunchDescription([vi_node])
