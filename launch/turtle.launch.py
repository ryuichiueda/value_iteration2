import launch, os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    tb3_gazebo_share_dir = get_package_share_directory("turtlebot3_gazebo")
    emcl2_share_dir = get_package_share_directory("emcl2")
    rviz_config_dir = os.path.join(get_package_share_directory('value_iteration2'), 'rviz', 'config.rviz')

    tb3_gazebo_launch = launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource([tb3_gazebo_share_dir + "/launch/turtlebot3_world.launch.py"]),
    )
    emcl2_launch = launch.actions.IncludeLaunchDescription(
      PythonLaunchDescriptionSource([emcl2_share_dir + "/test/test.launch.xml"]),
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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir + '/config/config.rviz'])

    launch_include = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("emcl2"),
                emcl2_share_dir + "/test/test.launch.xml",
            )
        )
    )

    return launch.LaunchDescription([
        launch_include
    #    tb3_gazebo_launch,
        #emcl2_launch,
    #    vi_node,
    #    rviz_node,
    ])
