# What a client asks of one motor, in joint space (rad, rad/s, Nm at the
# mechanism). Mirrors drlabs_core::lowlevel::MotorCmd without its padding.
uint8 MODE_OFF=0
uint8 MODE_DRIVE=1

uint8 mode      # MODE_OFF: observe, torque off. MODE_DRIVE: follow q/dq.
float32 q       # target position, rad
float32 dq      # target velocity, rad/s
float32 kp      # stiffness, Nm/rad
float32 kd      # damping, Nm*s/rad
float32 tau     # feed-forward torque, Nm
