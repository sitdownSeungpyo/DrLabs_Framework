# A client's command for every axis. Only the first n entries mean anything,
# but the array is fixed so the message is one size.
uint32 FLAG_QUICK_STOP=1    # latch the robot-wide fault now (CiA 402 quick stop)
uint32 FLAG_FAULT_RESET=2   # clear it; every axis is OFF afterwards

uint32 seq        # increments with every record; a repeat is the same command
uint32 flags      # FLAG_*
uint64 t_ns       # the client's clock when it built this record, informational
uint16 n          # how many of cmd are in use
MotorCmd[64] cmd
