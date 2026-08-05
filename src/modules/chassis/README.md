# Chassis module

The active 1 kHz control path is intentionally kept in this directory root:

- `chassis_module`: feedback acquisition, safety gates, actuator output and telemetry.
- `stool_controller`: startup pose generation and joint cascade PID.
- `body_motion_estimator`: SPR-style forward-motion state estimation.
- `leg_kinematics`: five-bar forward/inverse kinematics.
- `leg_vmc`: leg-length control and virtual-force-to-joint-torque mapping.
- `lqr_schedule`: leg-length-dependent balance gains.

Component-specific tuning stays in the corresponding `.cpp` file instead of
being collected in a global parameter header.

`legacy/` contains the superseded controller stack. It is retained only to
preserve previous development work and is excluded from the firmware build.
