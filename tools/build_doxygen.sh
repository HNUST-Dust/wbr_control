#!/usr/bin/env bash
## @file build_doxygen.sh
#  @brief 生成 wbr_control HTML API 文档并汇总 Doxygen 警告。
#  @details 脚本固定从仓库根目录运行 Doxygen，成功后打印首页和警告日志的绝对路径。

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
DOXYGEN_BIN="${DOXYGEN_BIN:-doxygen}"

if ! command -v "${DOXYGEN_BIN}" >/dev/null 2>&1; then
	echo "doxygen is not installed or DOXYGEN_BIN is invalid" >&2
	exit 127
fi

cd "${PROJECT_DIR}"

# Homebrew Doxygen 1.18.0 on macOS can sporadically terminate with SIGBUS while
# parsing this C++ tree. Retry only that process-level failure; syntax and
# documentation warnings still fail immediately below.
DOXYGEN_STATUS=0
for ATTEMPT in 1 2 3 4 5; do
	set +e
	"${DOXYGEN_BIN}" Doxyfile
	DOXYGEN_STATUS=$?
	set -e
	if [[ ${DOXYGEN_STATUS} -eq 0 ]]; then
		break
	fi
	if [[ ${DOXYGEN_STATUS} -ne 138 || ${ATTEMPT} -eq 5 ]]; then
		exit "${DOXYGEN_STATUS}"
	fi
	echo "Doxygen received SIGBUS; retrying (${ATTEMPT}/5)" >&2
done

WARNING_LOG="${PROJECT_DIR}/build/doxygen-warnings.log"
HTML_INDEX="${PROJECT_DIR}/build/doxygen/html/index.html"

if [[ ! -f "${HTML_INDEX}" ]]; then
	echo "Doxygen did not generate ${HTML_INDEX}" >&2
	exit 1
fi

if [[ -s "${WARNING_LOG}" ]]; then
	WARNING_COUNT="$(wc -l < "${WARNING_LOG}" | tr -d ' ')"
	echo "Doxygen warnings: ${WARNING_COUNT} (showing first 40)"
	sed -n '1,40p' "${WARNING_LOG}"
	exit 1
fi

echo "Doxygen HTML: ${HTML_INDEX}"
echo "Doxygen warnings: ${WARNING_LOG}"
