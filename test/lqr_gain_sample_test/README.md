# lqr_gain_sample_test

Physical system-identification data logger for LQR fitting.

This test no longer samples `EvaluateLqrGain()`. Instead, it visits each leg
length working point, logs the measured state vector and the injected input
sequence, and leaves K calculation to the host-side fitting script.

Default safety posture:

- DM output is disabled.
- wheel output is disabled.
- wheel PRBS perturbation is disabled.
- leg-angle PRBS perturbation is disabled.

Enable outputs only after the robot is on a safe test rig.

## Build

```bash
cmake -S wbr_control/test/lqr_gain_sample_test \
  -B wbr_control/test/lqr_gain_sample_test/build \
  -GNinja \
  -DBOARD=hpm6750evk2 \
  -DPython3_EXECUTABLE=/Users/panpoming/Documents/zephyr_projects/.venv/bin/python

CCACHE_DISABLE=1 ninja -C wbr_control/test/lqr_gain_sample_test/build
```

## Output

The test prints:

```text
lqr_sysid_csv,t_ms,point_mm,phase,sample,theta_mrad,theta_rate_mradps,x_mm,x_rate_mms,pitch_mrad,pitch_rate_mradps,u_wheel_mNm,u_leg_mNm,len_l_mm,len_r_mm,roll_mrad,yaw_rate_mradps,valid
```

State mapping for fitting:

- `x0 = theta`
- `x1 = theta_rate`
- `x2 = x`
- `x3 = x_rate`
- `x4 = pitch`
- `x5 = pitch_rate`

Input mapping:

- `u0 = u_wheel`
- `u1 = u_leg`

All logged state/input values are integer-scaled by `1e3`.

Filter serial logs by keeping `lqr_sysid_csv` lines and removing the first
field. Feed the resulting CSV to `tools/fit_lqr_from_sysid.py` to identify
per-leg-length discrete K values, then use `tools/fit_lqr_schedule.py` to fit
the final polynomial schedule.
