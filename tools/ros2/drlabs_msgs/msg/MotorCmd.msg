# What a client asks of one motor, in joint space (rad, rad/s, Nm at the
# mechanism). Mirrors drlabs_core::lowlevel::MotorCmd without its padding.
uint8 MODE_OFF=0          # observe, torque off
uint8 MODE_DRIVE=1        # position q, dq and tau as feed-forward
uint8 MODE_VELOCITY=2     # run at dq, no position target, tau feed-forward
uint8 MODE_TORQUE=3       # tau only
uint8 MODE_HOLD=4         # zero-velocity servo (a position servo holds where it is)
uint8 MODE_STAY_WITHIN=5  # torque only outside the joint's declared travel
uint8 MODE_ZERO=6         # once, on the change into this mode: output position becomes q

uint8 mode      # one of MODE_*; see docs/RT.md
float32 q       # target position, rad
float32 dq      # target velocity, rad/s
float32 kp      # stiffness, Nm/rad
float32 kd      # damping, Nm*s/rad
float32 tau     # feed-forward torque, Nm
