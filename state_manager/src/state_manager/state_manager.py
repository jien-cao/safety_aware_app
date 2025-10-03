# copyright

from threading import Lock

from .state_definitions import SystemState


class StateManager:
    """Spped Control System State Manager"""

    def __init__(self):
        self._lock = Lock()
        self._state = SystemState.FULL_SPEED

    def update_from_sensor(self, reading: float) -> None:
        """Update from sensor reading(s).

        Args:
            reading (float): sensor reading.
        """
        state = self._checker(reading)
        with self._lock:
            self._state = state

    def get_state(self) -> SystemState:
        """Get current system state.

        Returns:
            SystemState: current state.
        """
        with self._lock:
            return self._state

    def _checker(reading: float) -> SystemState:
        """Checker implemented for vicinity logic.

        TODO: a chain of checkers could be encapsulated in this method.

        Args:
            reading (float): sensor reading, unit: mm.

        Returns:
            SystemState: the corresponding state.
        """
        if reading < 400.0:
            return SystemState.STOP
        if reading < 800.0:
            return SystemState.SLOW
        return SystemState.FULL_SPEED
