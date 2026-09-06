# wheel_current_mapping_test

Suspended-wheel test for identifying the relationship between the raw C620
current command and the raw current feedback.

## Safety

- The complete robot must be securely suspended with both wheels clear.
- Keep the remote enable switch in the disabled position while powering up.
- Enabling starts one complete test. Disabling, remote timeout, stale wheel
  feedback, or CAN transmit failure immediately commands zero.
- The maximum raw command is 2000. With the documented C620 scale
  `16384 counts = 20 A`, this is approximately 2.44 A and remains below the
  provisional 2.5 A limit.
- Leg motors are never enabled or commanded by this application.

After a completed run, disable and re-enable the remote switch to repeat it.

## VOFA+ JustFloat channels

Only six channels are emitted:

1. `command_raw`
2. `feedback_current_raw`
3. `motor_feedback_rpm_raw`
4. `estimated_gearbox_output_rpm`
5. `estimated_gearbox_output_accel_rad_s2`
6. `stage`

Stage values `1..99` are left-wheel steps. Stage values `101..199` are
right-wheel steps. Stage `999` means the run is complete. Stage `0` means
disabled or waiting for fresh feedback.

The command sequence for each wheel is:

```text
0,
+200, 0, -200, 0,
+400, 0, -400, 0,
+800, 0, -800, 0,
+1200, 0, -1200, 0,
+1600, 0, -1600, 0,
+2000, 0, -2000, 0
```

Non-zero pulses last 600 ms. Zero intervals last at least 800 ms and are
automatically extended until the feedback speed is close to zero, so the
application never applies a reverse pulse to a rapidly spinning wheel. There
is an additional 1000 ms zero-command settling interval before each wheel
starts.

## Build

```bash
cmake -S wbr_control/test/wheel_current_mapping_test \
  -B wbr_control/test/wheel_current_mapping_test/build \
  -GNinja \
  -DBOARD=dust-hpm6750 \
  -DPython3_EXECUTABLE=/Users/panpoming/Documents/zephyr_projects/.venv/bin/python

CCACHE_DISABLE=1 ninja -C wbr_control/test/wheel_current_mapping_test/build
```

Firmware:

```text
wbr_control/test/wheel_current_mapping_test/build/zephyr/zephyr.bin
```

## Interpretation

The raw feedback RPM is the value reported directly by the M3508/C620
protocol. Because the encoder is associated with the motor, the test treats
this as motor-side RPM and also emits the estimated gearbox-output RPM:

```text
output_rpm = feedback_rpm * 17 / 268
```

The specified reduction ratio is:

```text
N = 268 / 17 = 15.7647058824
```

The raw and reduced values are both retained so this assumption can be
checked against the measured no-load speed. The ratio converts speed and
reflected inertia, but speed plus reduction ratio alone cannot determine
torque. Torque additionally requires either:

- a trustworthy output torque constant versus feedback current;
- known effective inertia and angular acceleration plus a friction model; or
- direct wheel-rim force measurement.

The test therefore logs angular acceleration so an inertia-based torque fit
can be added once wheel, rotor, and gearbox inertias are available.
