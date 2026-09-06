#!/usr/bin/env bash
## @file build_local.sh
#  @brief 使用当前 Zephyr 工作区构建 wbr_control 主固件。
#  @details 该工具在主机侧运行，用于构建、采集、辨识或参数生成；不会编译进目标固件。生成参数写回固件前应按对应文档完成单位和符号约定检查。

set -euo pipefail

# Build wbr_control into its local build directory.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_DIR="$(cd "${APP_DIR}/.." && pwd)"

BUILD_DIR="${APP_DIR}/build"
BOARD="${BOARD:-dust-hpm6750}"
WEST_BIN="${WORKSPACE_DIR}/../.venv/bin/west"
CCACHE_TEMPDIR="${CCACHE_TEMPDIR:-${TMPDIR:-/tmp}/wbr_control_ccache}"

mkdir -p "${CCACHE_TEMPDIR}"
export CCACHE_TEMPDIR

if [[ ! -x "${WEST_BIN}" ]]; then
  WEST_BIN="west"
fi

cd "${WORKSPACE_DIR}"

if [[ $# -gt 0 ]]; then
  "${WEST_BIN}" build -p always -b "${BOARD}" -s "${APP_DIR}" -d "${BUILD_DIR}" -- -DCONF_FILE="$1"
else
  "${WEST_BIN}" build -p always -b "${BOARD}" -s "${APP_DIR}" -d "${BUILD_DIR}"
fi
