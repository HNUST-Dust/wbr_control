# Board Layer

This layer contains adapters that express board capabilities in application
terms.

Examples:

- `board_led.c`
- `board_button.c`
- `board_power.c`
- `board_identity.c`

This directory is private to `chassis_controller`; gimbal board adapters belong
under `gimbal_controller/platform/board/`.
