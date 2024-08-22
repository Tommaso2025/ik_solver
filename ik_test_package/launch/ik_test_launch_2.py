
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import LoadComposableNodes

def generate_launch_description():
    # Set the path to your URDF file
    #urdf_xacro_file = PathJoinSubstitution(
    #    [FindPackageShare('panda_moveit_config'), 'urdf', 'panda.urdf.xacro']
    #)
        
    #srdf_file = PathJoinSubstitution(
    #    [FindPackageShare('panda_moveit_config'), 'srdf', 'panda.srdf']
    #)
    
    # Path to the URDF, SRDF and yaml files in the src directory
    urdf_xacro_file = os.path.join(
        os.getenv('HOME'), 'ws_moveit_2/src/panda_moveit_config/config/panda.urdf.xacro'
    )
    srdf_file = os.path.join(
        os.getenv('HOME'), 'ws_moveit_2/src/panda_moveit_config/config/panda.srdf'
    )
    kinematics_yaml = os.path.join(
        os.getenv('HOME'), 'ws_moveit_2/src/panda_moveit_config/config/kinematics.yaml'
    )
        
        # Load the kinematics.yaml file content
    with open(kinematics_yaml, 'r') as file:
        kinematics_params = yaml.safe_load(file)
                
    return LaunchDescription([
        # Declare the launch argument to allow passing a different URDF file path
        DeclareLaunchArgument(
            'urdf_file',
            default_value=urdf_xacro_file,
            description='Absolute path to robot URDF file'
        ),
        DeclareLaunchArgument(
            'srdf_file',
            default_value=srdf_file,
            description='Absolute path to robot SRDF file'
        ),
        # Define the node that will use the URDF file
        Node(
            package='ik_test_package',
            executable='ik_test_node_2',
            name='ik_test_node_2',
            output='screen',
            respawn=False,  # Ensure respawn is disabled
            parameters=[
		    {'robot_description': ParameterValue(Command(['xacro ', LaunchConfiguration('urdf_file')]), value_type=str)},
		    {'robot_description_semantic': ParameterValue(Command(['cat ', srdf_file]), value_type=str)},
            	    kinematics_params
	    ],
	    #arguments=['--ros-args', '--log-level', 'DEBUG']
        )
    ])
    
if __name__ == '__main__':
    generate_launch_description()



