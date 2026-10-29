
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # Paquete de descripcion del UR5 + gripper
    description_pkg = Path(
        get_package_share_directory("ur5_robotiq_description")
    )

    # Paquete con la configuracion de MoveIt
    moveit_pkg = Path(
        get_package_share_directory("ur5_gripper_moveit_config")
    )

    # Modelo cinemático utilizado por MoveIt.
    # No incluye ground_plane ni el plugin de Gazebo.
    robot_xacro = (
        description_pkg
        / "urdf"
        / "ur_with_gripper_moveit.urdf.xacro"
    )

    # Configuracion de MoveIt
    moveit_config = (
        MoveItConfigsBuilder(
            "ur",
            package_name="ur5_gripper_moveit_config"
        )
        .robot_description(
            file_path=str(robot_xacro)
        )
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml"
        )
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # Servidor de planificacion MoveIt
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
    )

    # RViz para visualizar la planificacion
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            str(moveit_pkg / "config" / "moveit.rviz")
        ],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {"use_sim_time": True},
        ],
    )

    # Gazebo inicia por separado:
    # - controller_manager
    # - robot_state_publisher
    # - joint_state_broadcaster
    #
    # Este launch no debe duplicar esos nodos.

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
