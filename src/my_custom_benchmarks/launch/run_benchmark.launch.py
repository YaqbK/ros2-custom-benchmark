import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Construct the absolute path to the URDF in the ur_description package
    urdf_file = os.path.join(
        get_package_share_directory('ur_description'),
        'urdf',
        'ur.urdf.xacro'
    )

    # Load the UR5e robot configuration
    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .robot_description(
            file_path=urdf_file,
            mappings={"ur_type": "ur5e", "name": "ur"}
        )
        .robot_description_semantic(
            file_path="srdf/ur.srdf.xacro",
            mappings={"ur_type": "ur5e", "name": "ur"}
        )
        .trajectory_execution(file_path="config/controllers.yaml") # Fix controller warning
        .planning_pipelines(pipelines=["ompl"]) # Ignore Pilz, only load OMPL
        .to_moveit_configs()
    )

    # Path to your YAML config
    benchmark_config = os.path.join(
        get_package_share_directory('my_custom_benchmarks'),
        'config',
        'benchmark_config.yaml'
    )

    # Your custom C++ node
    benchmark_node = Node(
        package='my_custom_benchmarks',
        executable='custom_benchmark_node',
        name='custom_benchmark_node',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            benchmark_config
        ],
    )

    return LaunchDescription([benchmark_node])
