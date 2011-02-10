pr_msgs/Joints[] positions
float32[] blend_radius
uint32 options

# options should be powers of 2, so that they can be OR'd together
uint32 opt_WaitForStart=1
uint32 opt_HoldOnStall=2
uint32 opt_HoldOnForceInput=4
#uint32 opt_          =8  # placeholder for next value
