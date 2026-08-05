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

接口头文件与对应驱动实现放在同一目录树中，并通过 `<platform/...>` 引用。

Implementation files in `platform/` may depend on Zephyr, HPM SDK, and public
channel contracts, but must not depend on business modules.

The chassis-to-CAN transmit call is an explicit real-time exception to
message-only module communication. Chassis may call the stable
`SubmitCanStandardFrame()` API directly; it must not include or depend on the
CAN dispatch implementation.
