import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    # Get the launch directory
    pkg_share = FindPackageShare(package='fishbot_cartographer').find('fishbot_cartographer')

    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # map resolution
    resolution = LaunchConfiguration('resolution', default='0.05')

    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    configuration_directory = LaunchConfiguration('configuration_directory', default=os.path.join(pkg_share, 'config'))

    # config file 
    configuration_basename = LaunchConfiguration('configuration_basename', default='fishbot_2d.lua')

    # declare the node 

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory, '-configuration_basename', configuration_basename])

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
        # arguments=['-d', os.path.join(pkg_share, 'rviz', 'demo.rviz')])
    ld = LaunchDescription()
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)
    
    return ld
