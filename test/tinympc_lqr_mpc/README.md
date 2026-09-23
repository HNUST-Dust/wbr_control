# TinyMPC chassis experiment

This is a host-only experiment. It is intentionally not included by the
production Zephyr CMake files.

The library snapshot is fetched from `NoneOfEver/TinyMPC` at the unchanged
upstream-compatible commit `023f36bd5b27267e1a96ed47a774d120b5707f13`.
It is managed by the manifest repository's `west.yml` and checked out as
`TinyMPC` at the workspace root.

It uses the existing 6-state, 2-input physical LQR model at the generated leg
length closest to 0.25 m, discretizes it at the production 1 kHz rate, and
compares:

- the current continuous-LQR gain followed by actuator saturation;
- TinyMPC with the same Q/R weights and torque constraints inside the QP.

The exact wheel constraint follows the production conversion and protocol
limit: each wheel is limited to `16384 / 3450` Nm, hence the model's total
wheel input is limited to twice that value. The total body-on-leg input is
conservatively limited to 8 Nm. This is not yet an exact representation of the
four 54 Nm joint limits: those also depend on leg Jacobians, support force and
the independent left/right coordination term. A firmware candidate must turn
those into configuration-dependent, time-varying linear constraints.

Build and run:

```sh
python3 test/tinympc_lqr_mpc/generate_model.py
cmake -S test/tinympc_lqr_mpc -B /tmp/wbr-tinympc-test-build
cmake --build /tmp/wbr-tinympc-test-build -j
/tmp/wbr-tinympc-test-build/wbr_tinympc_lqr_mpc_test
```

Host timing is useful only for comparing configurations. It is not the MCU
WCET; the generated embedded solver must be benchmarked with the target cycle
counter and production memory placement.

## Current host result

With an 80 ms horizon and three initial-condition scenarios, all 4500
closed-loop MPC calls converged in at most 13 ADMM iterations and the returned
first input stayed inside both torque bounds. Compared with LQR followed by
saturation, the constrained MPC reduced the integrated squared state error in
all three scenarios. On the development Mac, 2000 warm solves measured about
0.55 ms mean and 0.61 ms p99. The roughly 7 ms observed maximum is host-OS
scheduling noise and is not a target WCET measurement.

The checked-out convenience API and its generated source both retain
`Matrix<Dynamic, Dynamic>` and heap allocation, so upstream code generation by
itself does not make this revision MCU-safe. The application therefore maintains
`src/modules/chassis/mpc/static_tinympc_solver.hpp`, a compile-time-sized,
input-box-only online kernel. The host equivalence regression uses `6 x 2 x 81`
with `double`; the production shadow benchmark uses `6 x 2 x 21` with `float`.
Its Riccati cache is generated offline into
`generated_chassis_model.hpp`; setup, resizing and factorization are absent
from the online call.

The static kernel is built with `EIGEN_RUNTIME_NO_MALLOC`. Every online solve
runs while `Eigen::internal::set_is_malloc_allowed(false)` is active, so an
attempted Eigen heap allocation aborts the test. A separate
`wbr_tinympc_static_smoke` executable does not link the upstream dynamic
TinyMPC library at all. Its fixed `6 x 2 x 81` double workspace is 27,040 bytes
after eliminating the two previous-consensus trajectory copies.

The static and upstream implementations currently agree for every simulated
step in all three scenarios, including iteration counts. Their offline cache
differences are at floating-point roundoff level (`Kinf` about `3.6e-14`,
`Pinf` about `4.7e-9`). On the development Mac, warm static solves measured
about 2.2 us mean. This is still not an MCU WCET measurement.

Run the allocation smoke test with:

```sh
ctest --test-dir /tmp/wbr-tinympc-test-build --output-on-failure
```

For eventual firmware use, instantiate the solver in static storage rather
than on the 1 kHz thread stack. This reduced kernel intentionally supports only
the current zero-reference regulator with input box constraints; state,
general-linear and time-varying VMC constraints need their own fixed-size
storage and projection paths before those features are enabled.
