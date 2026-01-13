import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the package directory
    pkg_drone_gazebo = get_package_share_directory('drone_gazebo')
    
    # Path to the world file
    world_file = os.path.join(pkg_drone_gazebo, 'worlds', 'simple_world.sdf')
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    verbose = LaunchConfiguration('verbose', default='false')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_verbose_cmd = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Set to true to enable verbose output'
    )
    
    # Start Gazebo with the world file
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen',
        shell=False
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add declarations
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_verbose_cmd)
    
    # Add Gazebo server
    ld.add_action(gazebo_server)
    
    return ld
