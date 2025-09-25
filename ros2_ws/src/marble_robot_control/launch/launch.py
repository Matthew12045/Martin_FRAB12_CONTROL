import shutil

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    wheel_separation = LaunchConfiguration("wheel_separation")
    wheel_radius = LaunchConfiguration("wheel_radius")
    max_wheel_speed = LaunchConfiguration("max_wheel_speed")
    use_receiver = LaunchConfiguration("use_receiver")
    use_keyboard = LaunchConfiguration("use_keyboard")
    keyboard_in_terminal = LaunchConfiguration("keyboard_in_terminal")

    declare_wheel_separation = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.18",
        description="Distance between the wheels in meters.",
    )
    declare_wheel_radius = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
        description="Wheel radius in meters (used by controller node for kinematics).",
    )
    declare_max_wheel_speed = DeclareLaunchArgument(
        "max_wheel_speed",
        default_value="0.8",
        description="Maximum wheel linear speed in meters per second.",
    )
    declare_use_receiver = DeclareLaunchArgument(
        "use_receiver",
        default_value="false",
        description="Set true to launch the debugging receiver_node alongside the controller.",
    )
    declare_use_keyboard = DeclareLaunchArgument(
        "use_keyboard",
        default_value="true",
        description="Set false to skip launching the keyboard teleop publisher.",
    )
    gnome_terminal_path = shutil.which("gnome-terminal")
    terminal_available = gnome_terminal_path is not None

    declare_keyboard_in_terminal = DeclareLaunchArgument(
        "keyboard_in_terminal",
        default_value="true" if terminal_available else "false",
        description="Launch the keyboard teleop inside its own gnome-terminal window.",
    )

    controller_node = Node(
        package="marble_robot_control",
        executable="controller_node",
        name="controller_node",
        output="screen",
        parameters=[
            {
                "wheel_separation": wheel_separation,
                "wheel_radius": wheel_radius,
                "max_wheel_speed": max_wheel_speed,
            }
        ],
    )

    receiver_node = Node(
        package="marble_robot_control",
        executable="receiver_node",
        name="receiver_node",
        output="screen",
        condition=IfCondition(use_receiver),
    )


    keyboard_terminal_node = Node(
        package="marble_robot_control",
        executable="eiei_node",
        name="keyboard_publisher",
        output="screen",
        prefix=f"{gnome_terminal_path} -- " if terminal_available else "",
        condition=IfCondition(
            PythonExpression([
                "'",
                use_keyboard,
                "' == 'true' and '",
                keyboard_in_terminal,
                "' == 'true' and '",
                "true" if terminal_available else "false",
                "' == 'true'",
            ])
        ),
    )

    keyboard_inline_node = Node(
        package="marble_robot_control",
        executable="eiei_node",
        name="keyboard_publisher",
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'",
                use_keyboard,
                "' == 'true' and ('",
                keyboard_in_terminal,
                "' == 'false' or '",
                "false" if terminal_available else "true",
                "' == 'true')",
            ])
        ),
    )

    actions = [
        declare_wheel_separation,
        declare_wheel_radius,
        declare_max_wheel_speed,
        declare_use_receiver,
        declare_use_keyboard,
        declare_keyboard_in_terminal,
    ]

    if not terminal_available:
        actions.append(
            LogInfo(
                msg=(
                    "gnome-terminal not found; launching keyboard teleop in the current shell. "
                    "Set keyboard_in_terminal:=false to suppress this message or install gnome-terminal."
                )
            )
        )

    actions.extend([
        controller_node,
        receiver_node,
        keyboard_terminal_node,
        keyboard_inline_node,
    ])

    ld = LaunchDescription(actions)

    return ld
