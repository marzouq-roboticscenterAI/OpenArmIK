# Physical range recordings

`openarm_hand_range_calibration.json` is produced atomically after every Stop
and explicit Save by the compiled, read-only `openarm_calibrate_gui` tool. It
stores the complete timestamped raw and unwrapped encoder path for each manually
swept motor, not only endpoints.

For this installed pair, `can0` is robot-right and `can1` is robot-left. The
operator moves each torque-disabled joint from relaxed to one chosen safe
limit, through relaxed to the other chosen safe limit, and back to relaxed.
Mechanical stops must not be forced.
