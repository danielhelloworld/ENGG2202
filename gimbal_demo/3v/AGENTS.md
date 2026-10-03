# F32C Version 3 接手入口

- 默认中文沟通。处理本目录前读取仓库根目录的 `docs/GIMBAL_CONTEXT.md`、`docs/handoffs/gimbal-zero.md`，再读本目录 `README.md`、`GUI_README.md`、`MECHANICAL_ZERO.md`。
- 缓存是有日期的状态快照，分支、源码和设备状态以实际检查为准；其他电脑使用仓库相对路径，COM 号重新确认。
- GUI 默认机械参考；CLI/共享驱动默认软件参考；独立 K230 示例尚未升级固定零点，不能混淆。
- 保留旧版目录和其他模块改动。缓存读取、测试和代码同步不自动启动电机或写入设备参数。
- PC 回归命令：在本目录运行 `python -B -m unittest discover -s . -p 'test_*.py' -v`。模拟验证与实机验证分别记录。
