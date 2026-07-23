# domain/modules

语义：领域业务模块。

当前实现位置：
- modules

目录约定：
- active 模块：直接位于 `modules/*`（如 `chassis`、`remote_input`）
- staged 模块：统一位于 `modules/staging/*`

活动模块共享单层 `modules` 命名空间，目录只负责组织文件，不再映射成子命名空间。
