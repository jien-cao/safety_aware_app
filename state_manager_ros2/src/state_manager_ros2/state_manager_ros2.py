# copyright

from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy, LivelinessPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import Range  # NOTE: selected type is for demonstration only

from state_manager import SystemState, StateManager
from state_msgs.msg import SystemState as SystemStateMsg


class StateManagerRos2(Node):
    """ROS2 adapater for the state manager."""

    def __init__(self):
        super().__init__("state_manager_ros2")
        self.declare_parameter("sensor_topic", "vicinity_sensor")
        self.declare_parameter("state_topic", "system_state")
        self.declare_parameter("state_publish_frequency", 100.0)  # Hz
        self.declare_parameter("liveliness_allowance", 5)  # number of missed heartbeats
        self.declare_parameter("qos_depth", 1)
        sensor_topic = self.get_parameter("sensor_topic").get_parameter_value().string_value
        state_topic = self.get_parameter("state_topic").get_parameter_value().string_value
        liveliness_allowance = self.get_parameter("liveliness_allowance").get_parameter_value().integer_value
        qos_depth = self.get_parameter("qos_depth").get_parameter_value().integer_value
        if (publish_frequency := self.get_parameter("state_publish_frequency").get_parameter_value().double_value) == 0:
            raise ValueError("state_publish_frequency must be non-zero.")
        if liveliness_allowance <= 0:
            raise ValueError("liveliness_allowance must be positive.")

        liveliness_duration = liveliness_allowance / publish_frequency  # i.e. allow N missed heartbeats.
        ros_duration = Duration(seconds=int(liveliness_duration), nanoseconds=int((liveliness_duration % 1) * 1e9))

        # NOTE: Heavy reliance on ROS2 specifics here, in particular:
        # - use (TRANSIENT_LOCAL, RELIABLE) for latched topics;
        # - use Liveliness as a heartbeat mechanism.
        # - with the understanding that ROS2 optimizes for intra-process communication, IFF (below):
        # TODO: ensure high-cricial workers are launched using ROS2 Component Container mechanism.
        qos_profile = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=qos_depth,
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=ros_duration,
        )

        self._state_manager = StateManager()
        self._state_publisher = self.create_publisher(
            msg_type=SystemStateMsg, topic=state_topic, qos_profile=qos_profile
        )
        self._timer = self.create_timer(timer_period_sec=1.0 / publish_frequency, callback=self._publish_callback)

        self.create_subscription(
            msg_type=Range,
            topic=sensor_topic,
            callback=self._sensor_callback,
            qos_profile=10,
        )

    def _sensor_callback(self, msg: Range) -> None:
        self._state_manager.update_from_sensor(msg.range)

    def _publish_callback(self) -> None:
        state = self._state_manager.get_state()
        msg = SystemStateMsg()
        msg.reason = "registered sensor: vicinity_sensor"
        match state:
            case SystemState.FULL_SPEED:
                msg.state = SystemStateMsg.FULL_SPEED
            case SystemState.SLOW:
                msg.state = SystemStateMsg.SLOW
            case SystemState.STOP:
                msg.state = SystemStateMsg.STOP
            case _:
                raise ValueError(f"Unexpected state: {state}")
        self._state_publisher.publish(msg)
