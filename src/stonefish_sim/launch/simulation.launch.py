from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitution import Substitution
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)

class ConcatenateSubstitutions(Substitution):
    def __init__(self, *substitutions):
        self.substitutions = substitutions

    def perform(self, context):
        return "".join([sub.perform(context) for sub in self.substitutions])


def generate_launch_description():
    stonefish_sim_dir = get_package_share_directory("stonefish_sim")
    stonefish_ros2_dir = get_package_share_directory("stonefish_ros2")

    simulation_data_default = PathJoinSubstitution([stonefish_sim_dir, "data"])

    simulation_data_arg = DeclareLaunchArgument(
        "simulation_data",
        default_value=simulation_data_default,
        description="Path to the simulation data folder",
    )

    scenario_desc_arg = DeclareLaunchArgument(
        "scenario",
        description="Path to the scenario file",
        #scenario files need to be added here
        choices=[
            "gbr_keyboard_demo",
            "gbr_pipeline",
            "gbr_docking",
            "gbr_structure",
            "gbr_DYNSYS2025",
            "vard",
        ],
    )

    window_res_x_arg = DeclareLaunchArgument(
        "window_res_x", default_value="1920", description="Window resolution width"
    )

    window_res_y_arg = DeclareLaunchArgument(
        "window_res_y", default_value="1080", description="Window resolution height"
    )

    quality_arg = DeclareLaunchArgument(
        "rendering_quality",
        default_value="high",
    )

    scenario_desc_resolved = PathJoinSubstitution(
        [
            stonefish_sim_dir,
            "scenarios",
            ConcatenateSubstitutions(
                LaunchConfiguration("scenario"), TextSubstitution(text=".scn")
            ),
        ]
    )

    include_stonefish_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [stonefish_ros2_dir, "/launch/stonefish_simulator.launch.py"]
        ),
        launch_arguments={
            "simulation_data": LaunchConfiguration("simulation_data"),
            "scenario_desc": scenario_desc_resolved,
            "window_res_x": LaunchConfiguration("window_res_x"),
            "window_res_y": LaunchConfiguration("window_res_y"),
            "rendering_quality": LaunchConfiguration("rendering_quality"),
        }.items(),
    )

    return LaunchDescription(
        [
            simulation_data_arg,
            scenario_desc_arg,
            window_res_x_arg,
            window_res_y_arg,
            quality_arg,
            include_stonefish_launch,
        ]
    )