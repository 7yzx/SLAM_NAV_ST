import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution

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

    # # URDF 文件路径
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'backpack_2d.urdf')
    # with open(urdf_file_path, 'r') as infp:  # 打开URDF文件
    #         robot_desc = infp.read()  # 读取URDF文件内容
            
    # 启动 robot_state_publisher，发布 URDF 描述的 IMU 和 LiDAR 坐标系
    robot_state_publisher_node = Node(
        package='robot_state_publisher',  # 指定节点所在的包
        executable='robot_state_publisher',  # 指定可执行文件
        name='robot_state_publisher',  # 指定节点名称
        output='screen',  # 输出到屏幕
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file_path])  # 传递URDF文件路径作为参数

    

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('imu/data','imu')],
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
        output='screen'
        # arguments=['-d', os.path.join(pkg_share, 'rviz', 'demo_2d.rviz')]
    )
        # arguments=['-d', os.path.join(pkg_share, 'rviz', 'demo.rviz')])
    ld = LaunchDescription()
    ld.add_action(robot_state_publisher_node)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz_node)
    
    return ld
