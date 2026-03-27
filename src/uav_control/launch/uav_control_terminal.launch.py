import os
import shutil
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
import launch_ros.actions
import launch_ros.descriptions
from ament_index_python.packages import get_package_share_directory


def _get_interactive_terminal_prefix():
    # 按常见环境优先级选择可用终端
    if shutil.which('gnome-terminal'):
        return 'gnome-terminal --'
    if shutil.which('xterm'):
        return 'xterm -e'
    if shutil.which('konsole'):
        return 'konsole -e'
    if shutil.which('xfce4-terminal'):
        return 'xfce4-terminal -x'
    return None

def generate_launch_description():

    px4_msg_bridge = launch_ros.actions.Node(
        package='uav_control',
        executable='px4_msg_bridge',
        name='px4_msg_bridge',
        output='screen'
    )

    so3_config_file = os.path.join(
        get_package_share_directory('so3_control'),
        'config',
        # 'sunray150_mid360_config.yaml'
        'gz_x500_config.yaml'
    )

    so3_control_component = launch_ros.descriptions.ComposableNode(
        package='so3_control', 
        plugin='SO3ControlComponent',
        name='so3_control_component',
        parameters=[so3_config_file],
        )
    
    so3_control_container = ComposableNodeContainer(
            name='so3_control_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                so3_control_component
            ],
            output='screen'
        )
    
    terminal_config_file = os.path.join(
        get_package_share_directory('uav_control'),
        'config',
        'control_terminal_config.yaml'
    )
    
    uav_control_terminal = launch_ros.actions.Node(
            package='uav_control',
            executable='uav_control_terminal',
            name='uav_control_terminal',
            output='screen',
            emulate_tty=True,
            prefix=_get_interactive_terminal_prefix(),
            parameters=[terminal_config_file,so3_config_file]
        )

    ld = LaunchDescription()
    ld.add_action(px4_msg_bridge)
    ld.add_action(so3_control_container)
    ld.add_action(uav_control_terminal)

    return ld