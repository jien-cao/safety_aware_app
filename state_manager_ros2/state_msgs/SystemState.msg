# copyright

# ENUM to mirror the SystemState definition
# FULL_SPEED = auto()
# SLOW = auto()
# STOP = auto()

uint8 FULL_SPEED=0
uint8 SLOW=1
uint8 STOP=2

uint8 state       # enum type of SystemState (above)
string reason ""  # optionally provide details
