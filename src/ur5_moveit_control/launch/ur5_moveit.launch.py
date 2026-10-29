
from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder(
            "ur",
            package_name="ur5_gripper_moveit_config"
        )
        .to_moveit_configs()
    )

    ur5_moveit_node = Node(
        package="ur5_moveit_control",
        executable="ur5_moveit_node",
        name="ur5_moveit_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([
        ur5_moveit_node
    ])
