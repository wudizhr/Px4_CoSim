from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
import launch_ros.actions
import launch_ros.descriptions

def generate_launch_description():
    so3_control_component = launch_ros.descriptions.ComposableNode(
            package='so3_control', 
            plugin='SO3ControlComponent',
            name='so3_control_component',
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
    
    so3_control_example = launch_ros.actions.Node(
            package='so3_control',
            executable='control_example',
            name='control_example',
            output='screen'
        )
    
    px4_so3_example = launch_ros.actions.Node(
            package='uav_control',
            executable='px4_so3_example',
            name='px4_so3_example',
            output='screen'
        )

    ld = LaunchDescription()
    ld.add_action(so3_control_container)
    ld.add_action(so3_control_example)
    ld.add_action(px4_so3_example)
    return ld