# copyright

from enum import Enum, auto

class SystemState(Enum):
    """System State for Speed Control"""
    FULL_SPEED = auto()
    SLOW = auto()
    STOP = auto()
