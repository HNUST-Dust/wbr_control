# Driver Adapters

This layer contains application-owned wrappers around low-level devices or
external modules.

Examples:

- `motor_driver.c`
- `imu_port.c`
- `can_port.c`
- `display_port.c`

Keep Zephyr device acquisition and low-level transactions here when possible.

Application-facing adapters share the single `platform` namespace.

接口头文件与对应驱动实现放在同一目录树中，并通过
`<chassis_controller/platform/...>` 引用。

Implementation files in `chassis_controller/platform/` may depend on Zephyr, HPM SDK, and public
channel contracts, but must not depend on business modules.

Threaded communication sessions and message routing belong to
`src/chassis_controller/communication/`; this directory is limited to hardware
adaptation.
