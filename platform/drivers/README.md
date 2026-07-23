# Driver Adapters

This layer contains application-owned wrappers around low-level devices or
external modules.

Examples:

- `motor_driver.c`
- `imu_port.c`
- `can_port.c`
- `display_port.c`

Keep Zephyr device acquisition and low-level transactions here when possible.

Application-facing adapters share the single `platform` namespace. Public
functions carry a component name, such as `InitializeCanDispatch()` and
`InitializeUsbSession()`, so flattening the namespace does not create ambiguity.
