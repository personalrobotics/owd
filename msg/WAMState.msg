float64[] positions
# float64[] velocities  # not implemented yet
float64[] torques
TrajInfo[] trajectory_queue
TrajInfo prev_trajectory
uint8 state

uint8 state_free=0
uint8 state_fixed=1
uint8 state_moving=2
uint8 state_stalled=3
uint8 state_inactive=255
