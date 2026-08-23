"""Bring the bridge up and, unless told otherwise, drive it to active.

Every tunable is a launch argument, so nothing here has to be edited to point
at a different robot:

    ros2 launch drlabs_bridge bridge.launch.py host:=192.168.1.50 port:=7777

``autostart:=false`` leaves the node unconfigured, which is what you want when
something else — a supervisor, a mission node — owns when the axis goes live.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description() -> LaunchDescription:
    arguments = [
        DeclareLaunchArgument("host", default_value="127.0.0.1",
                              description="Address of the drlabs bringup server."),
        DeclareLaunchArgument("port", default_value="7777",
                              description="TCP port the bringup server listens on."),
        DeclareLaunchArgument("publish_rate", default_value="50.0",
                              description="Hz at which joint_states is published."),
        DeclareLaunchArgument("frame_id", default_value="",
                              description="frame_id stamped on joint_states."),
        DeclareLaunchArgument("namespace", default_value="",
                              description="Namespace for the bridge node."),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="Configure and activate the node on launch."),
    ]

    bridge = LifecycleNode(
        package="drlabs_bridge",
        executable="bridge_node",
        name="drlabs_bridge",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{
            "host": LaunchConfiguration("host"),
            "port": LaunchConfiguration("port"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            "frame_id": LaunchConfiguration("frame_id"),
        }],
    )

    autostart = IfCondition(LaunchConfiguration("autostart"))

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(bridge),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=autostart,
    )

    # Activate only once configuring has actually succeeded — emitting both
    # transitions at launch races the first one.
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=bridge,
            goal_state="inactive",
            entities=[EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(bridge),
                transition_id=Transition.TRANSITION_ACTIVATE,
            ))],
        ),
        condition=autostart,
    )

    return LaunchDescription(arguments + [bridge, activate, configure])
