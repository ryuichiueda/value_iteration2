import launch, os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    emcl2_share_dir = get_package_share_directory("emcl2")

    config = os.path.join(
      get_package_share_directory('value_iteration2'),
      'config',
      'params.yaml'
    )

    emcl2_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("emcl2"),
                emcl2_share_dir + "/test/test.launch.xml",
            )
        )
    )

    return launch.LaunchDescription([
        emcl2_launch,
    ])
