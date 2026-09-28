# Tracing

这里放 tracing 和主机侧调试工具接入说明。

当前决策：

- 默认不启用重量级 tracing
- 可通过 `conf/systemview.conf` 启用 SEGGER SystemView
- 如果后面需要开放格式 tracing，再考虑补 CTF 配置
