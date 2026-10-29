
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    description_pkg = Path(
        get_package_share_directory("ur5_robotiq_description")
    )

    bringup_pkg = Path(
        get_package_share_directory("ur5_robotiq_bringup")
    )

    robot_xacro = (
        description_pkg
        / "urdf"
        / "ur_with_gripper_moveit.urdf.xacro"
    )

    controllers_file = (
        bringup_pkg
        / "config"
        / "controllers_sg.yaml"
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "ur",
            package_name="ur5_gripper_moveit_config"
        )
        .robot_description(
            file_path=str(robot_xacro)
        )
        .to_moveit_configs()
    )

    ur5_node = Node(
        package="ur5_moveit_control",
        executable="ur5_moveit_node",
        name="ur5_moveit_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        ur5_node,
    ])
