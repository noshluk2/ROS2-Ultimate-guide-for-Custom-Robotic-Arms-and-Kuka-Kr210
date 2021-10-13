import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths

def generate_launch_description():
    package_share_dir = get_package_share_directory("r2_arm")
    urdf_file = os.path.join(package_share_dir, "urdf", "big_bazu.urdf")
    
    controller_file = os.path.join(package_share_dir, "config", "jtc.yaml")
    model_path, plugin_path, media_path = GazeboRosPaths.get_paths()
    env = {
        "GAZEBO_MODEL_PATH": model_path,
        "GAZEBO_PLUGIN_PATH": plugin_path,
        "GAZEBO_RESOURCE_PATH": media_path,
    }
    robot_description = {"robot_description": urdf_file}
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo","-s","libgazebo_ros_factory.so",],
                output="screen",
                additional_env=env,
            ),
            Node(
                package="gazebo_ros",
                node_executable="spawn_entity.py",
                arguments=["-entity","ur3","-b","-file", urdf_file,
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                arguments=[urdf_file],
            ),
            
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controller_file],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
            Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                ),

            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            )
        ]
    )