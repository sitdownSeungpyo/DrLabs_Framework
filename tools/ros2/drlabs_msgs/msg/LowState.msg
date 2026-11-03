# The whole machine, as of one bus tick.
uint32 seq            # increments with every record published
uint32 cmd_seq        # the LowCmd.seq this state was computed against
uint64 tick           # bus ticks since the runtime started
uint64 t_ns           # the runtime's monotonic clock at the end of the tick, ns
uint32 period_ns      # the configured period
uint32 deadline_miss  # ticks that started late, since start
uint16 estop_code     # non-zero while the fault is latched: the trip code
uint8 estop           # 1 while the fault is latched
uint16 n              # how many of state are in use
MotorState[64] state
