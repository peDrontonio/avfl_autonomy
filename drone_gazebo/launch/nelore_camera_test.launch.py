import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Start Gazebo with the nelore test world file
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-v', '4', world_file],
        output='screen',
        shell=False
    )
    
    # Bridge for camera image
    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo'
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Image view node to visualize camera
    image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=['/camera'],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add declarations
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes
    ld.add_action(gazebo_server)
    ld.add_action(bridge_camera)
    ld.add_action(image_view)
    
    return ld
