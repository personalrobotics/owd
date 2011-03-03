float64[] positions
float64[] jpositions
# float64[] velocities  # not implemented yet
float64[] torques
pr_msgs/TrajInfo[] trajectory_queue
pr_msgs/TrajInfo prev_trajectory
uint8 state

uint8 state_free=0
uint8 state_fixed=1
uint8 state_moving=2
uint8 state_stalled=3
uint8 state_inactive=255
