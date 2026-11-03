# One motor as the runtime reports it. A value the driver did not read is NaN.
float32 q         # rad
float32 dq        # rad/s
float32 tau       # Nm at the mechanism, as estimated
float32 current   # A
float32 voltage   # V
float32 temp      # degrees C, the hottest the controller reports
uint16 fault      # the controller's own fault register, 0 is clear
uint8 status      # 0 ok, 1 silent, 2 alert, 3 refused, 4 off, 5 tripped
uint8 blocked     # why a DRIVE request is not honoured, 0 if it is
