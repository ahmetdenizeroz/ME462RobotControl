from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("my_robot_cell")
        .robot_description(file_path="config/my_robot_cell.urdf.xacro")
        .robot_description_semantic(file_path="config/my_robot_cell.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="true",
        description="Enable RViz Next prompts",
    )

    text_arg = DeclareLaunchArgument(
        "text",
        default_value="ROMER",
        description="Text to trace",
    )

    zenpool_draw_node = Node(
        package="zenpool_draw_letter",
        executable="zenpool_draw_letter",
        name="zenpool_draw_letter",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"debug": LaunchConfiguration("debug")},
            {"text": LaunchConfiguration("text")},
        ],
    )

    return LaunchDescription([debug_arg, text_arg, zenpool_draw_node])
