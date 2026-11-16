# Nav3 总启动包预留目录

后续在此实现四模块及主仓库导航任务管理器的联合启动与统一参数配置。
导航目标 Action、任务状态和受管理路径已由独立的
[`nav3_navigation_manager`](../nav3_navigation_manager/README.md) 功能包实现。
总启动需将 SCAN 的路径输入设为 `/nav3/managed_path`，并启用定位许可检查。
