# printk_test

Minimal continuous printk validation app.

## Build

From workspace root:

```bash
source .venv/bin/activate
west build -p always -b hpm6e00evk -d build-printk-test applications/wbr_control/test/printk_test
```

## Flash

```bash
source .venv/bin/activate
west flash -d build-printk-test --skip-rebuild
```

## Capture log

```bash
./applications/wbr_control/tools/serial_log.sh /dev/cu.usbserial-11301
```
