import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths

def generate_launch_description():
    package_share_dir = get_package_share_directory("big_bazu")
    urdf_file = os.path.join(package_share_dir, "urdf", "big_bazu_.urdf")
    
    controller_file = os.path.join(package_share_dir, "config", "jtc.yaml")
    robot_description = {"robot_description": urdf_file}
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo","-s","libgazebo_ros_factory.so",],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                node_executable="spawn_entity.py",
                arguments=["-entity","big_bazu","-b","-file", urdf_file,],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                arguments=[urdf_file],
            ),
            ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
            ),

            ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_trajectory_controller'],
        output='screen'
            )
            # Node(
            #     package="controller_manager",
            #     executable="ros2_control_node",
            #     parameters=[robot_description, controller_file],
            #     output={
            #         "stdout": "screen",
            #         "stderr": "screen",
            #     },
            # ),
            # Node(
            # package="controller_manager",
            # executable="spawner.py",
            # arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            #     ),

            # Node(
            #     package="controller_manager",
            #     executable="spawner.py",
            #     arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
            # )
        ]
    )
