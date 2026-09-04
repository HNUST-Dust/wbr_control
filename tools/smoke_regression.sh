#!/usr/bin/env bash
## @file smoke_regression.sh
#  @brief 执行主固件和关键配置组合的冒烟回归检查。
#  @details 该工具在主机侧运行，用于构建、采集、辨识或参数生成；不会编译进目标固件。生成参数写回固件前应按对应文档完成单位和符号约定检查。

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
WS_DIR="$(cd "$ROOT_DIR/../.." && pwd)"
BUILD_DIR="$ROOT_DIR/build"
BOARD="hpm6750evk2"
TMP_BUILD_DIR="/tmp/wbr_control_smoke_can_off"
OVERLAY_FILE="/tmp/wbr_control_smoke_can_off.conf"
TMP_LOG_DIR="/tmp/wbr_control_smoke_logs"

PASS_COUNT=0

pass() {
  echo "[PASS] $1"
  PASS_COUNT=$((PASS_COUNT + 1))
}

fail() {
  echo "[FAIL] $1" >&2
  exit 1
}

run_cmd() {
  local title="$1"
  shift
  local log_file="$TMP_LOG_DIR/${title// /_}.log"
  if "$@" >"$log_file" 2>&1; then
    return 0
  fi

  echo "[FAIL] $title" >&2
  echo "---- tail of $log_file ----" >&2
  tail -n 120 "$log_file" >&2 || true
  exit 1
}

check_file_contains() {
  local file="$1"
  local pattern="$2"
  local title="$3"
  if rg -q "$pattern" "$file"; then
    pass "$title"
  else
    fail "$title (pattern: $pattern, file: $file)"
  fi
}

echo "== wbr_control smoke regression =="
echo "ROOT_DIR=$ROOT_DIR"
echo "WS_DIR=$WS_DIR"
rm -rf "$TMP_BUILD_DIR"
mkdir -p "$TMP_LOG_DIR"

echo "-- [1/5] Static contract checks"
check_file_contains "$ROOT_DIR/src/main.cpp" "platform::InitializeCanDispatch\\(" "main initializes CAN dispatch directly"
check_file_contains "$ROOT_DIR/src/modules/remote_input/remote_input_module.cpp" "uart_rx_enable\\(" "Remote input module owns UART RX"
check_file_contains "$ROOT_DIR/src/modules/module_base.h" "static void ThreadEntry" "ModuleBase owns the thread entry"
check_file_contains "$ROOT_DIR/src/modules/module_base.h" "virtual int Start\\(\\) = 0" "ModuleBase requires Start"
check_file_contains "$ROOT_DIR/src/modules/module_base.h" "virtual void RunLoop\\(\\) = 0" "ModuleBase requires RunLoop"
check_file_contains "$ROOT_DIR/src/modules/chassis/chassis_module.h" "public ModuleBase" "Chassis inherits ModuleBase"
check_file_contains "$ROOT_DIR/src/modules/chassis/chassis_module.h" "int Start\\(\\) override" "Chassis overrides Start"
check_file_contains "$ROOT_DIR/src/modules/chassis/chassis_module.h" "void RunLoop\\(\\) override" "Chassis overrides RunLoop"
check_file_contains "$ROOT_DIR/src/modules/remote_input/remote_input_module.cpp" "device_is_ready\\(" "Remote input checks hardware in Start"
check_file_contains "$ROOT_DIR/src/main.cpp" "static modules::ChassisModule chassis_module" "main owns the chassis instance"
check_file_contains "$ROOT_DIR/src/main.cpp" "chassis_module\\.Start\\(" "main starts chassis explicitly"
check_file_contains "$ROOT_DIR/src/main.cpp" "CONFIG_WBR_CONTROL_MODULE_REMOTE_INPUT" "Remote input module is config-gated"
check_file_contains "$ROOT_DIR/src/main.cpp" "CONFIG_WBR_CONTROL_MODULE_CHASSIS" "Chassis module is config-gated"
check_file_contains "$ROOT_DIR/src/main.cpp" "CONFIG_WBR_CONTROL_MODULE_REFEREE" "Referee module is config-gated"

echo "-- [2/5] Build default configuration"
run_cmd "build_default" cmake --build "$BUILD_DIR" -j8
pass "Default build succeeds"

ELF_FILE="$BUILD_DIR/zephyr/zephyr.elf"
if [[ -f "$ELF_FILE" ]]; then
  pass "Default ELF exists"
else
  fail "Default ELF missing: $ELF_FILE"
fi

echo "-- [3/5] Build CAN-off configuration"
cat > "$OVERLAY_FILE" <<EOF
CONFIG_WBR_CONTROL_RUNTIME_INIT_CAN=n
CONFIG_WBR_CONTROL_MODULE_CHASSIS=n
EOF

PYTHON_BIN="${WS_DIR}/.venv/bin/python"
if [[ -x "$PYTHON_BIN" ]]; then
  run_cmd "configure_can_off" cmake -S "$ROOT_DIR" -B "$TMP_BUILD_DIR" -GNinja -DBOARD="$BOARD" -DPython3_EXECUTABLE="$PYTHON_BIN" -DOVERLAY_CONFIG="$OVERLAY_FILE"
else
  run_cmd "configure_can_off" cmake -S "$ROOT_DIR" -B "$TMP_BUILD_DIR" -GNinja -DBOARD="$BOARD" -DOVERLAY_CONFIG="$OVERLAY_FILE"
fi

run_cmd "build_can_off" cmake --build "$TMP_BUILD_DIR" -j8
pass "CAN-off build succeeds"

CAN_OFF_ELF="$TMP_BUILD_DIR/zephyr/zephyr.elf"
if [[ -f "$CAN_OFF_ELF" ]]; then
  pass "CAN-off ELF exists"
else
  fail "CAN-off ELF missing: $CAN_OFF_ELF"
fi

echo "-- [4/5] Config gate checks"
if rg -q "^# CONFIG_WBR_CONTROL_RUNTIME_INIT_CAN is not set$|^CONFIG_WBR_CONTROL_RUNTIME_INIT_CAN=n$" "$TMP_BUILD_DIR/zephyr/.config"; then
  pass "CAN runtime init is disabled in CAN-off config"
else
  fail "CAN runtime init disable flag missing in $TMP_BUILD_DIR/zephyr/.config"
fi

echo "-- [5/5] Summary"
echo "Smoke regression passed with ${PASS_COUNT} checks."
