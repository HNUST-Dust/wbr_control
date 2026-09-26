# Gimbal controller

This directory is an independent Zephyr application for the gimbal controller.
Build it from the west workspace root with:

```sh
west build -p always -b dust-hpm6750 \
  -s wbr_control/src/gimbal_controller \
  -d wbr_control/build/gimbal_controller
```

The application currently contains only a minimal entry point. Add gimbal-only
sources and configuration here; keep code shared with the chassis controller in
the repository-level shared directories.
