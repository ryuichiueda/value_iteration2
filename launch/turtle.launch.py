import launch, os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    tb3_gazebo_share_dir = get_package_share_directory("turtlebot3_gazebo")

    tb3_gazebo_launch = launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource([tb3_gazebo_share_dir + "/launch/turtlebot3_world.launch.py"]),
        launch_arguments={
        }.items()
    )

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

    return launch.LaunchDescription([
        tb3_gazebo_launch,
        vi_node,
    ])
