from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration,
    PathJoinSubstitution, PythonExpression
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_mujoco_arg = DeclareLaunchArgument(
        "use_mujoco",
        default_value="false",
    )
    use_mujoco = LaunchConfiguration("use_mujoco")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husky_description"), "urdf", "husky.urdf.xacro"]
            ),
            " name:=husky",
            " prefix:=''",
            " use_mujoco:=", use_mujoco,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    config_husky_velocity_controller = PathJoinSubstitution(
        [FindPackageShare("husky_control"),
        "config",
        "control.yaml"],
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    node_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
                config_husky_velocity_controller,
                robot_description,
                {'mujoco_scene_xacro_path': PathJoinSubstitution([FindPackageShare("husky_description"), "mjcf", "husky_scene.xml.xacro"]) },
                {'mujoco_scene_xacro_args': " as_two_wheels:=false"},
                ],
        remappings=[
            ('/husky_velocity_controller/odom', '/odom'),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_husky_velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["husky_velocity_controller"],
        output="screen",
    )

    # Launch husky_control/control.launch.py which is just robot_localization.
    launch_husky_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'control.launch.py'])))

    # Launch husky_control/teleop_base.launch.py which is various ways to tele-op
    # the robot but does not include the joystick. Also, has a twist mux.
    launch_husky_teleop_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'teleop_base.launch.py'])))

    # Launch husky_control/teleop_joy.launch.py which is tele-operation using a physical joystick.
    launch_husky_teleop_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_control"), 'launch', 'teleop_joy.launch.py'])))


    # Launch husky_bringup/accessories.launch.py which is the sensors commonly used on the Husky.
    launch_husky_accessories = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
        [FindPackageShare("husky_bringup"), 'launch', 'accessories.launch.py'])))


    ld = LaunchDescription()
    ld.add_action(use_mujoco_arg)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_controller_manager)
    ld.add_action(spawn_controller)
    ld.add_action(spawn_husky_velocity_controller)
    ld.add_action(launch_husky_control)
    ld.add_action(launch_husky_teleop_base)
    ld.add_action(launch_husky_teleop_joy)
    ld.add_action(launch_husky_accessories)

    # microstrain IMU driver + ZUPT: real hardware only, launch if package is installed
    # TODO: if the mujoco is launched, do same thing as real hardware so that we can use EKF for state estimation in simulation as well (currently we just use robot_state_publisher with remapping to joint_states topic)
    real_hw = UnlessCondition(PythonExpression(["'", LaunchConfiguration('use_mujoco'), "' == 'true'"]))
    try:
        microstrain_launch = get_package_share_directory('microstrain_inertial_driver')
        imu_zupt_script    = get_package_share_directory('husky_control')
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(microstrain_launch + '/launch/microstrain_launch.py'),
            condition=real_hw,
        ))
        ld.add_action(ExecuteProcess(
            cmd=['python3', imu_zupt_script + '/scripts/imu_zupt.py'],
            output='screen',
            condition=real_hw,
        ))
    except PackageNotFoundError:
        pass

    return ld
