#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SOURCE_GLOBS=(-g '*.{c,cc,cpp,h,hpp}')
EXCLUDES=(-g '!build/**' -g '!test/**/build/**')

# PX4 风格下头文件跟随所有者，旧的统一 include 门面不得重新出现。
if find "${ROOT_DIR}/include" -type f -print -quit 2>/dev/null | rg -q .; then
  echo "Headers must follow their owning component; the top-level include tree must stay empty." >&2
  exit 1
fi

if rg -n \
  '#include [<"]wbr_control/' \
  "${ROOT_DIR}" \
  "${EXCLUDES[@]}" \
  "${SOURCE_GLOBS[@]}"; then
  echo "The legacy <wbr_control/...> include facade has been removed." >&2
  exit 1
fi

if rg -n \
  'PROJECT_SOURCE_DIR}/include|\\.\\./\\.\\./include|include/wbr_control' \
  "${ROOT_DIR}" \
  -g 'CMakeLists.txt' \
  -g '!build/**' \
  -g '!test/**/build/**'; then
  echo "CMake still references the removed include tree." >&2
  exit 1
fi

# 共享调度策略只有一个所有者，不得重新产生模块线程工具层。
if rg -n \
  'thread_utils\\.h|StartMemberThread' \
  "${ROOT_DIR}" \
  "${EXCLUDES[@]}" \
  -g '!docs/**' \
  -g '!tools/check_include_boundaries.sh'; then
  echo "Shared scheduling policy belongs under src/scheduling." >&2
  exit 1
fi

# channels 是最低层消息契约，不依赖业务模块、协议或平台实现。
if rg -n \
  '#include [<"](?:modules|protocols|platform)/|modules::|platform::|protocols::' \
  "${ROOT_DIR}/channels" \
  "${SOURCE_GLOBS[@]}"; then
  echo "channels must not depend on modules, protocols, or platform." >&2
  exit 1
fi

# protocols 只负责编解码，不依赖应用层。
if rg -n \
  '#include [<"](?:modules|channels|platform)/|modules::|channels::|platform::' \
  "${ROOT_DIR}/src/protocols" \
  "${SOURCE_GLOBS[@]}"; then
  echo "protocols must not depend on modules, channels, or platform." >&2
  exit 1
fi

# 控制器与估计器由业务模块拥有，不再建立通用 algorithms 层。
if [[ -d "${ROOT_DIR}/src/algorithms" ]]; then
  echo "src/algorithms must not be recreated; place code under its owning module." >&2
  exit 1
fi

if rg -n \
  '#include [<"]algorithms/' \
  "${ROOT_DIR}" \
  "${EXCLUDES[@]}" \
  "${SOURCE_GLOBS[@]}"; then
  echo "The removed algorithms include root must not be referenced." >&2
  exit 1
fi

# platform 位于 modules 下层，允许使用 channel，但不能反向依赖业务模块。
if rg -n \
  '#include [<"]modules/|modules::' \
  "${ROOT_DIR}/platform" \
  "${SOURCE_GLOBS[@]}"; then
  echo "platform must not depend on modules." >&2
  exit 1
fi

# main 只组合模块入口，不穿透模块内部控制器。
if rg -n \
  '#include [<"]modules/chassis/(body_motion_estimator|leg_kinematics|leg_vmc|lqr_schedule|stool_controller)\\.h[>"]' \
  "${ROOT_DIR}/src/main.cpp"; then
  echo "main may include module entry headers only." >&2
  exit 1
fi

# chassis 内部控制器只允许所属目录和显式白盒测试访问。
if rg -n \
  '#include [<"]modules/chassis/(body_motion_estimator|leg_kinematics|leg_vmc|lqr_schedule|stool_controller)\\.h[>"]' \
  "${ROOT_DIR}" \
  "${EXCLUDES[@]}" \
  -g '!test/**' \
  -g '!src/modules/chassis/**' \
  "${SOURCE_GLOBS[@]}"; then
  echo "Chassis implementation headers are private to src/modules/chassis." >&2
  exit 1
fi

echo "Include boundary check passed."
