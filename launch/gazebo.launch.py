import os
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path("tank_robot_desc"), "urdf", "tank.urdf.xacro")
    gazebo_launch = os.path.join(get_package_share_path("gazebo_ros"), "launch", "gazebo.launch.py")
    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)
    # IMPORTANT: For the above line, don't forget to add a single space after "xacro"

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )
    
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "Panzerkampfwagen_VI_Ausführung_E"]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch)
        ),
        spawn_entity_node
    ])
