import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():

    ld = launch.LaunchDescription()

    pkg_name = "mrs_lib"
    pkg_share_path = get_package_share_directory(pkg_name)

    UAV_TYPE=os.getenv('UAV_TYPE')

    ld.add_action(launch.actions.DeclareLaunchArgument("debug", default_value="false"))
    dbg_sub = launch.substitutions.PythonExpression(['"" if "false" == "', launch.substitutions.LaunchConfiguration("debug"), '" else "debug_roslaunch"'])

    namespace='nmspc1'
    ld.add_action(Node(
        package=pkg_name,
        executable='param_loader_example',
        namespace=namespace,
        name='param_loader_example',
        parameters=[
            pkg_share_path + '/config/params_example.yaml',
            {"uav_type": UAV_TYPE,
             "param_namespace.floating_number": 1.2}
        ],
        output='screen',
        prefix=dbg_sub,
    ))

    return ld
