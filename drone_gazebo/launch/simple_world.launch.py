"""Launch simple_world in Gazebo and RViz."""
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate a launch description for simple_world."""
    # Get the package directories
    pkg_drone_gazebo = get_package_share_directory('drone_gazebo')
    pkg_drone_description = get_package_share_directory('drone_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ardupilot_gazebo = get_package_share_directory('ardupilot_gazebo')
    
    # Path to the world file
    world_file = os.path.join(pkg_drone_gazebo, 'worlds', 'simple_world.sdf')
    
    # Path to models directories
    models_path = os.path.join(pkg_drone_gazebo, 'models')
    drone_models_path = os.path.join(pkg_drone_description, 'models')
    
    # Set environment variable for Gazebo to find models
    gz_sim_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if gz_sim_resource_path:
        gz_sim_resource_path = f"{models_path}:{drone_models_path}:{gz_sim_resource_path}"
    else:
        gz_sim_resource_path = f"{models_path}:{drone_models_path}"
    
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_sim_resource_path
    
    # Nelore robot (SITL + bridges) - launches after Gazebo starts
    nelore = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("drone_gazebo"),
                        "launch",
                        "robots",
                        "nelore.launch.py",
                    ]
                ),
            ]
        )
    )
    
    # Gazebo server
    gz_sim_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={
            'gz_args': f'-v4 -s -r {world_file}'
        }.items(),
    )
    
    # Gazebo GUI
    gz_sim_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{Path(pkg_ros_gz_sim) / "launch" / "gz_sim.launch.py"}'
        ),
        launch_arguments={'gz_args': '-v4 -g'}.items(),
    )
    
    # RViz
    pkg_ardupilot_gz_bringup = get_package_share_directory('ardupilot_gz_bringup')
    rviz_config = os.path.join(pkg_ardupilot_gz_bringup, 'rviz', 'drone_gazebo_simple_world.rviz')
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    
    # Create the launch description
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true', description='Open RViz.'
        ),
        gz_sim_server,
        gz_sim_gui,
        nelore,
        rviz,
    ])
