from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

def generate_launch_description():
    arm_desc = get_package_share_directory("arm_description")
    
    # Launch Arguments
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(arm_desc, "urdf", "arm_new.urdf.xacro"),
        description="Absolute path to robot URDF/Xacro file"
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="True")

    # Set Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=[str(Path(arm_desc).parent.resolve())]
    )

    # Process Xacro file
    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " is_ignition:=False"
        ]),
        value_type=str
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen"
    )

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={'gz_args': '-r -v 1 empty.sdf'}.items()
    )

    # Spawn robot
    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-x", "0", "-y", "0", "-z", "0.5"],
        output="screen"
    )

    # ROS-GZ bridge
    gz_ros2_gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen"
    )

    # Controller configuration
    controller_config = PathJoinSubstitution([
        FindPackageShare("arm_controller"),
        "config",
        "arm_controllers.yaml"
    ])

    # # Controller manager
    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[controller_config],  # Removed robot_description
    #     output="screen"
    # )

    # Spawner nodes
    joint_state_broadcaster_spawn = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--param-file", controller_config],
        output="screen"
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--param-file", controller_config],
        output="screen"
    )

    return LaunchDescription([
        model_arg,
        use_sim_time,
        gazebo_resource_path,
        robot_state_publisher,
        gazebo,
        gz_spawn,
        gz_ros2_gazebo_bridge,
        # controller_manager,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=gz_spawn,
                on_exit=[joint_state_broadcaster_spawn]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawn,
                on_exit=[arm_controller_spawner]
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[gripper_controller_spawner]
            )
        )
    ])