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
- `recovery_controller`: enable-edge fall recovery and startup-pose generation.
- `body_motion_estimator`: SPR-style forward-motion state estimation.
- `leg_kinematics`: five-bar forward/inverse kinematics.
- `leg_vmc`: leg-length control and virtual-force-to-joint-torque mapping.
- `support_force_estimator`: joint-feedback/IMU wheel-ground support-force estimate.
- `flight_controller`: passive liftoff handling and touchdown detection.
- `jump_controller`: active pre-compress, thrust, air-retract and landing sequence.
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

In balance mode, the remote input `leg_length_delta` is interpreted as a leg
length rate command. Pushing the right stick upward extends both legs, pulling
it downward retracts them, and releasing it holds the current reference. The
rate and minimum/maximum references are configured under `balance` in
`chassis_params.yaml`.

The chassis telemetry frame sends 20 channels: controller state; target, left/right
length and left/right length rate; measured roll, roll reference and roll compensation
force; pitch; estimated forward speed, target/measured yaw rate; left/right wheel
torque; left/right commanded axial force; left/right filtered support force; and one
multiplexed action-phase channel. Action phase uses `10 + Flight`, `20 + Jump`,
`30 + ClimbStairs`, or `40 + Recovery`; zero means no atomic action is active.

Balance applies proportional roll compensation as equal and opposite additions to
the two VMC axial support forces. The roll reference is synchronized on entry and
rate-limited back to level, while gain, force limit, and IMU-direction sign remain
runtime-generated parameters under `balance`.

Each balance leg receives one half of the robot gravity as its nominal VMC support
feedforward. The rate-limited leg-length reference also supplies its actual reference
rate to VMC, enabling the configured extension-velocity feedforward instead of relying
on length error and integral force to overcome gravity.

Passive `Flight` detection is enabled only in `Balance`. Both filtered support
forces must remain below 20% of the nominal single-side static load for 20 ms,
and both legs must be at least 0.17 m long so the minimum-length mechanical stop
cannot masquerade as liftoff. In `Air`, only the two leg-angle LQR state pairs
remain active; airborne wheel torque is zero and each airborne leg receives an
80 N extension force until close to maximum length. Touchdown uses the parallel
leg-speed reversal and extension-peak-drop tests with a 10 ms hold. After both
sides latch touchdown, references are synchronized back into `Balance`.

The stair-climb action is a single atomic `ClimbStairs` mode. It is started on
the rising edge of `RemoteInputState::climb_stairs`, drives only the four leg
joints through `Motion1` and `Motion2`, and returns to balance with synchronized
references. WFly requests the action when CH4 is in its high position; DR16 and
VT03 use the positive end of the wheel channel (greater than 0.8). A phase timeout enters
the latched action-fault state, which requires an explicit remote disable before
the chassis can be enabled again.

On WFly, CH5 high publishes the independent `RemoteInputState::jump` request.
The former CH4-low plus CH5-high launcher-control combination has been removed;
when CH4 is low, CH5 mid/high uses the normal drive/yaw controls while the existing
CH5-low disabled pose remains unchanged. Jump will consume only the request rising
edge and enforce its own cooldown. CH4 high remains stair climb.

The atomic `Jump` action is accepted only from `Balance`, on the CH5-high rising
edge, and only when its three-second cooldown has expired. A simultaneous passive
liftoff detection takes priority over the jump request. The phases are
`PreCompress`, `Thrust`, `AirRetract`, `Air`, and `Return`: balance control first
compresses both legs, a conservative open-loop joint torque performs the thrust,
then the legs retract with zero wheel torque. After the retract reference has
settled, the existing Flight controller performs airborne extension and touchdown
detection. The configured B/D thrust signs describe the left-side kinematic branch;
the controller applies each leg's branch sign so the mirrored right mechanism extends
in the same physical direction. Phase and total timeouts enter the latched action-fault state.

Every explicit remote-enable rising edge now enters the atomic `Recovery` mode
after DM arming and feedback validation. Its phases are `Init`, `TurnOver`,
`YawFront`, `Swing`, `DrawBack`, and `Return`; `YawFront` currently passes through
an explicit no-gimbal interface whose availability, alignment, and leg-motion
safety fields are ready for later wiring. Recovery is allowed to operate outside
the normal balance tilt envelope, while remote disable, stale IMU, missing motor
feedback, and DM-not-ready remain hard interrupts. A recovery timeout or repeated
turn-over failure enters the same latched action-fault policy as stair climbing.
The current HI91 path classifies the upright side from chassis Z acceleration.
`recovery.upright_accel_sign` normalizes the installed sensor direction and is
currently `+1`. An initially upright chassis goes directly to `DrawBack`; only
the fallen branch is permitted to enter `TurnOver`. The turnover sweep freezes
on the first upright sample and uses a short confirmation window; in the current
no-gimbal build it then enters `DrawBack` directly instead of adding a `Swing`.

`legacy/` contains the superseded controller stack. It is retained only to
preserve previous development work and is excluded from the firmware build.
