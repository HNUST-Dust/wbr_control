# Chassis module

The active 1 kHz control path is intentionally kept in this directory root.  The
files stay flat for now; names express the dependency boundary without adding
another directory hierarchy:

- `chassis_module`: 1 kHz thread lifecycle, cycle orchestration and frequently edited
  VOFA debug channel mapping.
- `chassis_types`: shared physical-domain snapshots and actuator requests.
- `chassis_params.yaml`: editable mechanics, safety, controller and hardware parameters.
- `chassis_params.schema.yaml`: parameter types, C++ names, ranges and cross-field constraints.
- `tools/generate_params.py`: repository-wide schema validation and constexpr generation.
- `chassis_config`: C++-specific mappings and values derived from generated parameters.
- `chassis_imu_adapter`: selected IMU channel and chassis-frame transformation.
- `chassis_input_reader`: channel snapshots, protocol decoding and freshness checks.
- `chassis_state_machine`: enable sequence, safety latches and operating-state policy.
- `balance_controller`: motion references, body-speed estimation, unified LQR and VMC.
- `chassis_actuator`: physical torque limiting, protocol packing and CAN submission.
- `stool_controller`: startup pose generation and joint cascade PID.
- `body_motion_estimator`: SPR-style forward-motion state estimation.
- `leg_kinematics`: five-bar forward/inverse kinematics.
- `leg_vmc`: leg-length control and virtual-force-to-joint-torque mapping.
- `lqr_schedule`: leg-length-dependent balance gains.

## Parameter workflow

Edit `chassis_params.yaml` and build normally. CMake validates it against
`chassis_params.schema.yaml` and generates
`build/generated/params/chassis_params_generated.h`; the generated header
is not a source file and must not be edited or committed. Parameter generation
has no firmware runtime cost.

The schema owns each parameter's C++ constant name, scalar type, bounds and
allowed values, plus relationships such as minimum < maximum. The Python
generator is chassis-parameter agnostic: adding a parameter only requires an
entry in the value YAML and a matching entry in the schema YAML. It rejects
missing or unknown fields and invalid values. PyYAML is supplied by the Python
environment used by west/Zephyr.

Algorithm-internal coefficients that are not normal tuning parameters remain
with their component. The large unified LQR schedule continues to use its
dedicated derivation script and generated coefficient table.

`legacy/` contains the superseded controller stack. It is retained only to
preserve previous development work and is excluded from the firmware build.
