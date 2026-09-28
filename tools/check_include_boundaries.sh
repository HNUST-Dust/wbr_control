#!/usr/bin/env bash
## @file check_include_boundaries.sh
#  @brief 检查模块头文件归属和跨层包含边界。
#  @details 该工具在主机侧运行，用于构建、采集、辨识或参数生成；不会编译进目标固件。生成参数写回固件前应按对应文档完成单位和符号约定检查。

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
  echo "Shared scheduling mechanisms belong in src/periodic_schedule.h." >&2
  exit 1
fi

# msg 是最低层消息契约，不依赖业务模块、协议或平台实现。
if rg -n \
  '#include [<"](?:modules|protocols|platform|chassis_controller|gimbal_controller)/|modules::|platform::|protocols::' \
  "${ROOT_DIR}/msg" \
  "${SOURCE_GLOBS[@]}"; then
  echo "msg must not depend on modules, protocols, or platform." >&2
  exit 1
fi

# protocols 只负责编解码，不依赖应用层。
if rg -n \
  '#include [<"](?:modules|msg|platform|chassis_controller|gimbal_controller)/|modules::|msg::|platform::' \
  "${ROOT_DIR}/src/protocols" \
  "${SOURCE_GLOBS[@]}"; then
  echo "protocols must not depend on modules, msg, or platform." >&2
  exit 1
fi

# The shared scheduler contains mechanisms only. Application priorities and
# phase offsets live with the owning controller.
if rg -n \
  'thread_priorit|thread_phase|chassis_controller|gimbal_controller' \
  "${ROOT_DIR}/src/periodic_schedule.h"; then
  echo "src/periodic_schedule.h must remain controller-independent." >&2
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

# Application-owned platform code sits below business modules. It may include
# its own platform API, but not controller modules or the other application.
if rg --pcre2 -n \
  '#include [<"](?:chassis_controller/(?!platform/)|gimbal_controller/)|modules::' \
  "${ROOT_DIR}/src/chassis_controller/platform" \
  "${SOURCE_GLOBS[@]}"; then
  echo "chassis platform must not depend on controller modules or gimbal." >&2
  exit 1
fi

if rg --pcre2 -n \
  '#include [<"](?:gimbal_controller/(?!platform/)|chassis_controller/)|modules::' \
  "${ROOT_DIR}/src/gimbal_controller/platform" \
  "${SOURCE_GLOBS[@]}"; then
  echo "gimbal platform must not depend on controller modules or chassis." >&2
  exit 1
fi

# main 只组合模块入口，不穿透模块内部控制器。
if rg -n \
  '#include [<"]chassis_controller/modules/chassis/(body_motion_estimator|leg_kinematics|leg_vmc|lqr_schedule|stool_controller)\\.h[>"]' \
  "${ROOT_DIR}/src/chassis_controller/main.cpp"; then
  echo "main may include module entry headers only." >&2
  exit 1
fi

# chassis 内部控制器只允许所属目录和显式白盒测试访问。
if rg -n \
  '#include [<"]chassis_controller/modules/chassis/(body_motion_estimator|leg_kinematics|leg_vmc|lqr_schedule|stool_controller)\\.h[>"]' \
  "${ROOT_DIR}" \
  "${EXCLUDES[@]}" \
  -g '!test/**' \
  -g '!src/chassis_controller/modules/chassis/**' \
  "${SOURCE_GLOBS[@]}"; then
  echo "Chassis implementation headers are private to src/chassis_controller/modules/chassis." >&2
  exit 1
fi

echo "Include boundary check passed."
