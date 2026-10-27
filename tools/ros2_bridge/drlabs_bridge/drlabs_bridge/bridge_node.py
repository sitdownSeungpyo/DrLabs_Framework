"""A managed ROS 2 node that fronts a running ``drlabs rt`` runtime.

    ROS 2                                   drlabs bringup
    joint_commands  ──┐                  ┌──  JSON lines
                      ├─> bridge_node ──>┤     over TCP
    joint_states    <─┘                  └──  robot + safety envelope

The node is a lifecycle node because it manages hardware: an axis that is live
should be live because somebody activated it, and deactivating should release
it rather than leave it holding.  That is the whole reason to prefer the
managed form here.

It owns no control logic and no limits.  The envelope, the trips and the
controller watchdog all stay on the far side of the socket, where they belong —
this node translates, and is careful about what it claims.
"""

from __future__ import annotations

import math
import threading
from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState as JointStateMsg
from std_msgs.msg import String
from std_srvs.srv import Trigger

from drlabs_bridge.link import READY, RobotLink

#: Latched, so a subscriber that arrives late still learns the link state
#: instead of waiting for it to change.
LATCHED = QoSProfile(
    depth=1,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
)

#: Commands are reliable: dropping one silently is how an axis ends up
#: somewhere nobody asked for.
COMMAND_QOS = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.RELIABLE)


def _or_nan(value: Optional[float]) -> float:
    """Render a missing measurement as NaN rather than zero.

    JointState has no way to say "this joint reports no effort", and a
    consumer that reads a missing torque as zero torque will draw the wrong
    conclusion from it — the same reason the JSON protocol uses null.
    """
    return float("nan") if value is None else float(value)


class BridgeNode(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("drlabs_bridge")

        self.declare_parameter(
            "host", "127.0.0.1",
            ParameterDescriptor(description="Address of the drlabs rt runtime."))
        self.declare_parameter(
            "port", 7700,
            ParameterDescriptor(description="UDP port the runtime listens on."))
        self.declare_parameter(
            "publish_rate", 50.0,
            ParameterDescriptor(description=(
                "Hz at which joint_states is published.  State lines are buffered "
                "and flushed at this rate rather than republished as they arrive.")))
        self.declare_parameter(
            "frame_id", "",
            ParameterDescriptor(description="frame_id stamped on joint_states."))
        self.declare_parameter(
            "release_on_deactivate", True,
            ParameterDescriptor(description=(
                "Send a stop to every joint when deactivating.  Leave this on unless "
                "something else is taking over the axis in the same breath.")))

        self._link: Optional[RobotLink] = None
        self._latest: list = []
        self._latest_lock = threading.Lock()
        self._timer = None
        self._active = False

        # The command and service callbacks talk to the server and wait for a
        # reply, so they must not run on the same thread as the publish timer.
        self._blocking_group = ReentrantCallbackGroup()
        self._timer_group = MutuallyExclusiveCallbackGroup()

        self._states_pub = None
        self._connection_pub = None
        self._command_sub = None
        self._estop_srv = None
        self._reset_srv = None

    # --- lifecycle --------------------------------------------------------

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        host = self.get_parameter("host").value
        port = int(self.get_parameter("port").value)

        self._states_pub = self.create_lifecycle_publisher(
            JointStateMsg, "joint_states", qos_profile_sensor_data)
        self._connection_pub = self.create_lifecycle_publisher(
            String, "~/connection", LATCHED)
        self._command_sub = self.create_subscription(
            JointStateMsg, "joint_commands", self._on_command, COMMAND_QOS,
            callback_group=self._blocking_group)
        self._estop_srv = self.create_service(
            Trigger, "~/estop", self._on_estop, callback_group=self._blocking_group)
        self._reset_srv = self.create_service(
            Trigger, "~/reset", self._on_reset, callback_group=self._blocking_group)

        self._link = RobotLink(
            host=host, port=port,
            on_state=self._on_robot_state,
            on_connection=self._on_connection_change)

        self.get_logger().info(f"configured for {host}:{port}")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        if self._link is None:
            return TransitionCallbackReturn.ERROR

        rate = float(self.get_parameter("publish_rate").value)
        if rate <= 0:
            self.get_logger().error("publish_rate must be positive")
            return TransitionCallbackReturn.ERROR

        self._link.start()
        self._active = True
        self._timer = self.create_timer(
            1.0 / rate, self._publish_states, callback_group=self._timer_group)

        # The link comes up asynchronously; activation means "start trying",
        # not "the robot is there".  ~/connection is where that is reported.
        self.get_logger().info("activated; connecting")
        return super().on_activate(state)

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self._active = False
        if self._timer is not None:
            self._timer.cancel()
            self.destroy_timer(self._timer)
            self._timer = None
        if self._link is not None:
            release = bool(self.get_parameter("release_on_deactivate").value)
            self._link.stop(release=release)
        self.get_logger().info("deactivated")
        return super().on_deactivate(state)

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        self._teardown()
        return TransitionCallbackReturn.SUCCESS

    def _teardown(self) -> None:
        self._active = False
        if self._link is not None:
            self._link.stop(release=True)
            self._link = None
        for handle, destroy in (
            (self._timer, self.destroy_timer),
            (self._command_sub, self.destroy_subscription),
            (self._estop_srv, self.destroy_service),
            (self._reset_srv, self.destroy_service),
            (self._states_pub, self.destroy_publisher),
            (self._connection_pub, self.destroy_publisher),
        ):
            if handle is not None:
                destroy(handle)
        self._timer = None
        self._command_sub = None
        self._estop_srv = self._reset_srv = None
        self._states_pub = self._connection_pub = None
        with self._latest_lock:
            self._latest = []

    # --- from the robot ---------------------------------------------------

    def _on_robot_state(self, joints: list) -> None:
        """Runs on the link's reader thread: store, do not publish."""
        with self._latest_lock:
            self._latest = joints

    def _on_connection_change(self, state: str) -> None:
        self.get_logger().info(f"link {state}")
        if self._connection_pub is not None and self._active:
            message = String()
            message.data = state
            self._connection_pub.publish(message)

    def _publish_states(self) -> None:
        if not self._active or self._states_pub is None:
            return
        with self._latest_lock:
            joints = self._latest
        if not joints:
            return

        message = JointStateMsg()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.get_parameter("frame_id").value or ""
        message.name = [j.name for j in joints]
        message.position = [_or_nan(j.position_rad) for j in joints]
        message.velocity = [_or_nan(j.velocity_rad_s) for j in joints]
        message.effort = [_or_nan(j.torque_nm) for j in joints]
        self._states_pub.publish(message)

    # --- to the robot -----------------------------------------------------

    def _on_command(self, message: JointStateMsg) -> None:
        if not self._active or self._link is None or self._link.state != READY:
            self.get_logger().warn(
                "command ignored: the bridge is not active and connected",
                throttle_duration_sec=5.0)
            return
        if not message.name:
            self.get_logger().warn("command carries no joint names", throttle_duration_sec=5.0)
            return

        try:
            if message.position:
                targets = {
                    name: float(value)
                    for name, value in zip(message.name, message.position)
                    if not math.isnan(value)
                }
                if targets:
                    self._refuse_if_not_ok(self._link.set_positions(targets))
            elif message.velocity:
                for name, value in zip(message.name, message.velocity):
                    if not math.isnan(value):
                        self._refuse_if_not_ok(self._link.set_velocity(name, float(value)))
            else:
                self.get_logger().warn(
                    "command carries neither position nor velocity",
                    throttle_duration_sec=5.0)
        except (OSError, TimeoutError) as err:
            self.get_logger().error(f"command not delivered: {err}")

    def _refuse_if_not_ok(self, reply: dict) -> None:
        """A refusal is an answer, and it is the interesting one."""
        if not reply.get("ok", False):
            self.get_logger().warn(f"robot refused the command: {reply.get('error')}")

    def _on_estop(self, request, response):
        del request
        try:
            reply = self._link.estop("ROS 2 bridge") if self._link else {"error": "not configured"}
            response.success = bool(reply.get("ok", False))
            response.message = reply.get("error", "emergency stop latched")
        except (OSError, TimeoutError) as err:
            # The link is down, which is itself a stop: the controller's own
            # watchdog fires when commands stop arriving.  Say so rather than
            # reporting a success nobody confirmed.
            response.success = False
            response.message = f"link down ({err}); the controller watchdog applies"
        return response

    def _on_reset(self, request, response):
        del request
        try:
            reply = self._link.reset_estop() if self._link else {"error": "not configured"}
            response.success = bool(reply.get("ok", False))
            response.message = reply.get("error", "emergency stop cleared")
        except (OSError, TimeoutError) as err:
            response.success = False
            response.message = f"link down ({err})"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BridgeNode()
    # The command and service callbacks wait on the socket; the publish timer
    # must keep running while they do.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
