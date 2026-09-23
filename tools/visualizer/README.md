# WBR Architecture Browser

纯离线的消息拓扑与 Chassis 状态机浏览器。生成器扫描源码和 `msg/*.msg`，不依赖固件运行时。

```bash
python3 tools/visualizer/generate.py
```

直接用浏览器打开 `tools/visualizer/dist/index.html`。页面内嵌生成的数据，不需要 HTTP 服务或网络连接。生成目录 `dist/` 可以删除并随时重新生成。

当前扫描器识别消息存储对象的直接方法调用。通过指针、包装函数或运行期选择存储对象的关系不会被猜测；后续应通过显式扫描规则或标注补充。
