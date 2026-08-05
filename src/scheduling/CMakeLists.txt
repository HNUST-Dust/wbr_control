# 跨模块共享的线程优先级和周期调度策略。
add_library(wbr_scheduling INTERFACE)
target_include_directories(wbr_scheduling INTERFACE ${PROJECT_SOURCE_DIR}/src)
