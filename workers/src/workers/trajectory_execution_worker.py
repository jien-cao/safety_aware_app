# copyright

import math
import time
from threading import Lock

from state_manager import SystemState


class TrajectoryExecutionWorker:
    """Dummy trajectory execution worker.

    NOTE: mimick a behavior that's similar to the simulation tool's send_trajectory tool.
    """

    TRAJECTORY_RADIUS = 0.5
    ANGULAR_SPEED = math.pi / 20  # unit: rad / s (i.e. 20 seconds for half circle)
    FULL_SPEED_RATIO = 1.0
    SLOW_SPEED_RATIO = 0.5
    STOP_SPEED_RATIO = 0.0

    def __init__(self):
        self._angular_pos = 0.0
        self._state = SystemState.FULL_SPEED
        self._speed_ratio = self.FULL_SPEED_RATIO
        self._last_call_time = time.time()
        self._lock = Lock()

    def step(self) -> list[float]:
        """Step to the next position.

        NOTE: for demo, return (x, y, z) and let caller construct the full pose.

        Returns:
            list[float]: (x, y, z) position, unit: meters.
        """
        delta_t = time.time() - self._last_call_time
        with self._lock:
            new_pos = self._angular_pos + self.ANGULAR_SPEED * self._speed_ratio * delta_t
            self._angular_pos = new_pos % (2 * math.pi)
            self._last_call_time = time.time()

        return [math.cos(new_pos) * self.TRAJECTORY_RADIUS, math.sin(new_pos) * self.TRAJECTORY_RADIUS, 0.0]

    def update_state(self, state: SystemState) -> None:
        """Update the current system state.

        Args:
            state (SystemState): current system state.
        """
        with self._lock:
            if state == self._state:
                return
        match state:
            case SystemState.FULL_SPEED:
                speed_ratio = self.FULL_SPEED_RATIO
            case SystemState.SLOW:
                speed_ratio = self.SLOW_SPEED_RATIO
            case SystemState.STOP:
                speed_ratio = self.STOP_SPEED_RATIO
            case _:
                raise ValueError(f"Unexpected state: {state}")
        with self._lock:
            self._state = state
            self._speed_ratio = speed_ratio
