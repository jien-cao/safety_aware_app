# copyright

from threading import Lock
from typing import Any

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from rclpy.qos_event import SubscriptionEventCallbacks

from state_manager import SystemState
from state_msgs.msg import SystemState as SystemStateMsg


class StateMonitorRos2:
    """ROS2 adapater for a re-usable state monitor.

    NOTE: the "re-usability" is for use cases where there are multiple workers,
    all needing the same state information.
    """

    def __init__(self, node: Node):
        self._lock = Lock()
        self._system_state = SystemState.FULL_SPEED
        self._reason = ""
        self._node = node
        self._node.declare_parameter("state_topic", "system_state")
        self._node.declare_parameter("state_subscribe_frequency", 100.0)  # Hz
        self._node.declare_parameter("liveliness_allowance", 5)  # number of missed heartbeats
        self._node.declare_parameter("qos_depth", 1)
        state_topic = self._node.get_parameter("state_topic").get_parameter_value().string_value
        liveliness_allowance = self._node.get_parameter("liveliness_allowance").get_parameter_value().integer_value
        qos_depth = self._node.get_parameter("qos_depth").get_parameter_value().integer_value
        if (
            subscribe_frequency := self._node.get_parameter("state_subscribe_frequency")
            .get_parameter_value()
            .double_value
        ) == 0:
            raise ValueError("state_subscribe_frequency must be non-zero.")
        if liveliness_allowance <= 0:
            raise ValueError("liveliness_allowance must be positive.")

        liveliness_duration = liveliness_allowance / subscribe_frequency  # i.e. allow N missed heartbeats.
        ros_duration = Duration(seconds=int(liveliness_duration), nanoseconds=int((liveliness_duration % 1) * 1e9))

        # NOTE: Heavy reliance on ROS2 specifics here, in particular:
        # - use (TRANSIENT_LOCAL, RELIABLE) for latched topics;
        # - use dealine as a heartbeat mechanism.
        # - with the understanding that ROS2 optimizes for intra-process communication, IFF (below):
        # TODO: ensure high-cricial workers are launched using ROS2 Component Container mechanism.
        # NOTE: consider MutuallyExclusiveCallbackGroup if state processing logic becomes too heavy.
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            deadline=ros_duration,
        )
        subscription_callbacks = SubscriptionEventCallbacks(deadline=self._deadline_callback)
        self._state_subsription = self._node.create_subscription(
            msg_type=SystemStateMsg,
            topic=state_topic,
            callback=self._state_callback,
            qos_profile=qos_profile,
            event_callbacks=subscription_callbacks,
        )

    def get_status(self) -> tuple[SystemState, str]:
        """Get current system state.

        Returns:
            SystemState: current state.
        """
        with self._lock:
            return self._system_state, self._reason

    def _state_callback(self, msg: SystemStateMsg) -> None:
        with self._lock:
            match msg.state:
                case SystemStateMsg.FULL_SPEED:
                    self._system_state = SystemState.FULL_SPEED
                case SystemStateMsg.SLOW:
                    self._system_state = SystemState.SLOW
                case SystemStateMsg.STOP:
                    self._system_state = SystemState.STOP
                case _:
                    raise ValueError(f"Unexpected state: {msg.state}")
            self._reason = msg.reason

    def _deadline_callback(self, _: Any) -> None:
        with self._lock:
            self._system_state = SystemState.STOP
            self._reason = "missed heartbeat"
