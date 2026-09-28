# Gimbal platform

This directory owns board adapters, device drivers, communication transports,
and storage backends used only by `gimbal_controller`.

Do not import implementations from `chassis_controller/platform`. Reusable,
hardware-independent codecs and data types belong in repository-level shared
directories such as `src/protocols` and `msg`.
