# printk_test

Minimal continuous printk validation app.

## Build

From workspace root:

```bash
CCACHE_DISABLE=1 ../.venv/bin/west build -p always \
  -b hpm6750evk2 \
  wbr_control/test/printk_test \
  -d wbr_control/test/printk_test/build
```

## Flash

```bash
../.venv/bin/west flash \
  -d wbr_control/test/printk_test/build \
  --skip-rebuild
```

## Capture log

```bash
BAUD=921600 ./wbr_control/tools/serial_log.sh /dev/cu.usbserial-11301
```
