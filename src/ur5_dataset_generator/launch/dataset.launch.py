from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml
import os
from ament_index_python.packages import get_package_share_directory

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    ur_type = "ur5"

    # 1. Manually compile URDF
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro"]), " ",
        "name:=ur", " ",
        "ur_type:=", ur_type,
    ])
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    # 2. Manually compile SRDF (This fixes your error)
    robot_description_semantic_content = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro"]), " ",
        "name:=ur", " ",
        "ur_type:=", ur_type,
    ])
    robot_description_semantic = {"robot_description_semantic": ParameterValue(robot_description_semantic_content, value_type=str)}

    # 3. Manually load Kinematics
    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Run the Node
    dataset_generator_node = Node(
        package="ur5_dataset_generator",
        executable="dataset_generator",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
        ],
    )

    return LaunchDescription([dataset_generator_node])