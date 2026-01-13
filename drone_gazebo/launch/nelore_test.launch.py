import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Get the package directories
    pkg_drone_gazebo = get_package_share_directory('drone_gazebo')
    pkg_drone_description = get_package_share_directory('drone_description')
    
    # Path to the world file
    world_file = os.path.join(pkg_drone_gazebo, 'worlds', 'nelore_test_world.sdf')
    
    # Path to models directory from both packages
    models_path_gazebo = os.path.join(pkg_drone_gazebo, 'models')
    models_path_description = os.path.join(pkg_drone_description, 'models')
    
    # Set environment variable for Gazebo to find models
    gz_sim_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    paths_to_add = [models_path_gazebo, models_path_description]
    
    for path in paths_to_add:
        if gz_sim_resource_path:
            gz_sim_resource_path = f"{path}:{gz_sim_resource_path}"
        else:
            gz_sim_resource_path = path
    
    os.environ['GZ_SIM_RESOURCE_PATH'] = gz_sim_resource_path
    
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
    
    # Start Gazebo with the nelore test world file
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
