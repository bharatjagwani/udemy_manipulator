
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
import os.path
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue 
from launch.substitutions import Command, LaunchConfiguration
from pathlib import Path

def generate_launch_description():
    arm_desc = get_package_share_directory("arm_description")
    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value=os.path.join(get_package_share_directory("arm_description"),"urdf", "arm_new.urdf.xacro"),
        description="Absolute path to robot URDF/Xacro file"
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name = "GZ_SIM_RESOURCE_PATH",
        value = [str(Path(arm_desc).parent.resolve())]
    )
    
    robot_description = ParameterValue(Command(
        ["xacro ", 
         LaunchConfiguration("model")
         ]),
         value_type=str)


    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable= "robot_state_publisher",
        parameters=[{"robot_description": robot_description,
                     "use_sim_time": True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )
    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic","robot_description","-x", "0", "-y", "0", "-z", "0.5"],
        output="screen"
    )
    gz_ros2_gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": True}],
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"] ,
        output="screen"
    )

    joint_state_broadcaster_spawn = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['joint_state_broadcaster'],
    )

    arm_controller_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['arm_controller']
    )

    gripper_controller_spawner = Node(
        package = 'controller_manager',
        executable = 'spawner',
        arguments = ['gripper_controller']
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_spawn,
        gz_ros2_gazebo_bridge,
        joint_state_broadcaster_spawn,
        arm_controller_spawner,
        gripper_controller_spawner
    ])
