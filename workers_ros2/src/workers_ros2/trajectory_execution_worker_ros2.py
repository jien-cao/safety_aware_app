# copyright

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from workers.trajectory_execution_worker import TrajectoryExecutionWorker
from state_manager_ros2.state_msgs.msg import SystemState as SystemStateMsg
from state_monitor_ros2.src.state_monitor_ros2 import StateMonitorRos2


class TrajectoryExecutionWorkerRos2(Node):
    """ROS2 adapater for the trajectory execution worker."""

    DEMO_POSE_FRAME_ID = "map"
    DEMO_POSE_ORIGIN = [0.5, 0.5, 0.5]  # x, y, z in meters

    def __init__(self):
        super().__init__("trajectory_execution_worker_ros2")
        self.declare_parameter("pose_topic", "commanded_pose")
        self.declare_parameter("publish_frequency", 2.0)  # Hz
        # NOTE: response_frequence needs to be documented properly, it's
        self.declare_parameter("response_frequency", 100.0)  # Hz
        pose_topic = self.get_parameter("pose_topic").get_parameter_value().string_value
        if (publish_frequency := self.get_parameter("publish_frequency").get_parameter_value().double_value) == 0:
            raise ValueError("publish_frequency must be non-zero.")
        if (response_frequency := self.get_parameter("response_frequency").get_parameter_value().double_value) == 0:
            raise ValueError("response_frequency must be non-zero.")

        self._worker = TrajectoryExecutionWorker()
        self._state_monitor = StateMonitorRos2(self)
        self._publisher = self.create_publisher(msg_type=PoseStamped, topic=pose_topic, qos_profile=10)
        self._action_timer = self.create_timer(timer_period_sec=1.0 / publish_frequency, callback=self._action_callback)
        self._response_timer = self.create_timer(
            timer_period_sec=1.0 / response_frequency, callback=self._response_callback
        )

    def _action_callback(self) -> None:
        pos = self._worker.step()
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.DEMO_POSE_FRAME_ID
        pose_msg.pose.position.x = pos[0] + self.DEMO_POSE_ORIGIN[0]
        pose_msg.pose.position.y = pos[1] + self.DEMO_POSE_ORIGIN[1]
        pose_msg.pose.position.z = pos[2] + self.DEMO_POSE_ORIGIN[2]
        pose_msg.pose.orientation.x = 0.0
        pose_msg.pose.orientation.y = 0.0
        pose_msg.pose.orientation.z = 0.0
        pose_msg.pose.orientation.w = 1.0
        self._publisher.publish(pose_msg)

    def _response_callback(self) -> None:
        state, reason = self._state_monitor.get_status()
        self._worker.update_state(state)
        # NOTE: Log only abnormal states, as ros2 log is very slow.
        if state != SystemStateMsg.FULL_SPEED:
            self.get_logger().info(f"System state: {state.name}, reason: {reason}")
