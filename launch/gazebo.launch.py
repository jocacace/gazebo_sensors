
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.substitutions import FindPackageShare
import time
def generate_launch_description():

    ld = LaunchDescription()
    xacro_path = 'urdf/model.urdf.xacro'

    robot_description = PathJoinSubstitution([
        get_package_share_directory('gazebo_sensors'),	
        xacro_path
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description':Command(['xacro ', robot_description])
        }]
    )

    # Spawn
    spawn_node = Node(package='ros_gz_sim', executable='create',
                 arguments=[
                    '-name', 'cube',
                    '-z', '0.5',
                    '-topic', '/robot_description'],
                 output='screen')

    
    ignition_gazebo_node = IncludeLaunchDescription( PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
                                        launch_arguments=[('gz_args', [' -r -v 4 empty.sdf'])])
   
    color_camera_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
			name = 'color_camera_bridge',
			output='screen',
			parameters=[{
				'use_sim_time': True
			}],
			arguments = [
				'/color_camera' + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image'
			],
			remappings = [
				('/color_camera', '/color_camera')
			])

    depth_camera_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
			name = 'depth_camera_bridge',
			output='screen',
			parameters=[{
				'use_sim_time': True
			}],
			arguments = [
				'/depth_camera' + '@sensor_msgs/msg/Image' + '[ignition.msgs.Image',
				'/depth_camera/points' + '@sensor_msgs/msg/PointCloud2' + '[ignition.msgs.PointCloudPacked'
			],
			remappings = [
				('/depth_camera', '/depth_camera'),
				('/depth_camera/points', '/depth_camera/points')
			])
	
    depth_cam_data2cam_link_tf = Node(package='tf2_ros',
                     executable='static_transform_publisher',
                     name='cam3Tolink',
                     output='log',
                     arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'camera_link', 'cube/base_link/d435_depth'])

    ld.add_action( robot_state_publisher_node )
    ld.add_action( spawn_node )
    ld.add_action( ignition_gazebo_node )
    ld.add_action( color_camera_bridge )
    ld.add_action( depth_camera_bridge )
    ld.add_action( depth_cam_data2cam_link_tf )
    return ld
