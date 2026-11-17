# Nav3 模块协作与主仓库联调流程

更新日期：2026 年 11 月 17 日

## 一、仓库分工

Nav3 采用一个主仓库和五个 Git 子模块仓库。各模块负责人在自己的仓库维护 `main`；主仓库负责人决定何时把模块的新提交纳入整机版本。当前约定直接在 `main` 上迭代，不设置长期开发分支，也不要求通过 PR 交接。

| 负责人 | 负责仓库 | 主要工作 |
| --- | --- | --- |
| 彦男 | `Nav3_Sensor_ws`、`Nav3_Localization_ws` | MID360、FAST-LIO、局部地图、全局定位、定位状态和导航许可。 |
| 楚昕 | `Nav3_Global_planner_ws`、`Nav3_Chassis_ws` | PCT 全局规划；Go2 速度桥接与底盘接口。底盘暂未接入主项目 bringup。 |
| 主仓库负责人 | `Nav3_Local_Planner_ws`、`Nav3_ws` | SCAN、导航任务管理器、统一启动、跨模块参数、整机联调与验收记录。 |

GitHub 协作权限只授予队员负责的模块仓库；主仓库及局部规划仓库由主仓库负责人维护。各仓库可禁止强制推送和删除 `main`，同时保留本流程所需的直接推送方式。

## 二、一次改动的交接顺序

### 1. 模块负责人修改并独立验证

队员先更新本地模块仓库的 `main`，确认工作区干净，再修改代码。独立验证至少覆盖本次改动影响的构建、节点启动和输入输出接口；硬件、地图或 GPU 条件不具备时，应明确写出未验证项。验证通过后用**中文提交说明**提交并推送模块仓库的 `main`。

```bash
git status --short
git pull --ff-only origin main
# 修改并完成该模块的验证后，核对将要提交的文件
git status --short
git add -A
git diff --cached --check
git diff --cached --stat
git commit -m "中文说明本次改动"
git push origin main
```

交接消息至少包含：仓库与提交号、改动目的、实际测试命令与结果、需要的地图或 bag、未验证的条件。若修改 Topic、消息字段、Service、QoS、TF、坐标系、单位、时间戳或启动参数，应逐项列出旧值和新值。

### 2. 主仓库负责人拉取并联调

队员发出交接消息后，先暂停向该模块继续推送；主仓库负责人完成本轮联调或退回结果后再继续。这样双方不会在同一个 `main` 上同时提交，`git pull --ff-only` 也能明确发现远端分叉。

在主工作空间检查对应子模块没有未保存改动，然后切到它的 `main` 并拉取。以下以传感器模块为例；定位或全局规划只需替换路径：

```bash
git -C modules/Nav3_Sensor_ws status --short
git -C modules/Nav3_Sensor_ws switch main
git -C modules/Nav3_Sensor_ws pull --ff-only origin main
scripts/prepare_non_ros.sh --check
scripts/build_ros.sh --jobs 4
source scripts/source_env.bash
ros2 launch nav3_bringup nav3.launch.py driver:=false rviz:=false
```

`driver:=false` 只能检查无实体雷达输入时的联合启动。传感器、首次定位和路径执行的验收还需按改动内容做实体或带完整输入的回放实验，并把结果写入[联调记录](联调记录.md)。`git submodule update` 可能使子模块处于分离 HEAD；在子模块中开发、提交前必须确认已切回 `main`。

### 3. 适配、验收与版本固定

如果联调需要修改模块代码，主仓库负责人直接在该子模块本地的 `main` 修改、验证、用中文提交，并**先推送子模块仓库**。如果需要修改 `nav3_bringup`、任务管理器、README 或报告，则在主仓库提交这些文件。所有验收通过后，主仓库还要提交更新后的子模块指针，再推送主仓库 `main`。没有模块源码改动时，也要提交新的子模块指针；仅拉取子模块不会更新其他人克隆到的主项目版本。

```bash
# 验收通过且子模块提交已在其远端后，在主仓库根目录执行
git add modules/Nav3_Sensor_ws report/联调记录.md
git commit -m "更新传感器模块版本与联调记录"
git push origin main
```

若联调未通过，记录问题与复现条件，通知模块负责人继续处理，**不提交主仓库的子模块指针**。这样模块仓库可以继续迭代，主仓库仍固定在上一次已验收的版本。主仓库的本地子模块工作区需要恢复时，先确认其中没有未提交改动，再用 `git submodule update --checkout -- <模块路径>` 回到主仓库记录的提交。

### 4. 队员同步主仓库负责人修改

主仓库负责人推送模块适配提交后，队员在自己的模块仓库执行 `git pull --ff-only origin main`，以远端 `main` 为下一轮开发基线。需要查看整机版本的队员，拉取主仓库后还应运行 `git submodule update --init --recursive`；该命令检出主仓库固定的模块提交，不会自动选最新模块 `main`。

## 三、共同遵守的接口与数据约定

1. 当前统一全局地图为主工作空间 `data/maps/building_split_space.pcd`，SHA-256 为 `c84fba59d8c6342aa72c5c53599152295e27948ff0e3317d95fd9f3c12c1ab55`。地图和 bag 不纳入 Git；需要测试的队员另行取得相同数据并核对哈希。更换地图或雷达安装姿态前，须同步评估定位候选、PCT 缓存及坐标定义。
2. 核心 TF 链为 `map → odom → base_link → lidar`。定位负责 `map → odom`，FAST-LIO 负责 `odom → base_link`；同一段 TF 不应由两个模块同时发布。
3. 传感器向定位和 SCAN 提供 `/fast_lio/local_map`、`/fast_lio/odom`；定位发布 `/localization/navigation_allowed`。任务管理器调用 `/pct/plan` 后发布 `/nav3/managed_path`，SCAN 只消费这条受管理路径并输出 `/cmd_vel`。跨模块修改应先告知受影响模块的负责人。
4. 当前没有接入底盘。`/cmd_vel` 输出、首次定位和整机导航应分别记录验收结果，不能以节点启动成功代替实际功能通过。

## 四、交接时的最低验收证据

| 改动范围 | 模块负责人提交的证据 | 主仓库联调重点 |
| --- | --- | --- |
| 传感器 | 雷达与 IMU 输入、里程计、局部地图、健康状态及停止过程。 | 话题频率、时间戳、`odom → base_link`、定位与 SCAN 能否接收。 |
| 定位 | 地图与候选配置、`LocalizationResult`、`map → odom`、导航许可。 | 首次定位是否 `applied=true`，许可是否随数据失效而撤销。 |
| PCT | `/pct/publish_cost_map` 与 `/pct/plan` 的成功和失败样例。 | 地图一致性、路径 `map` 坐标和三维高度、管理器调用结果。 |
| 底盘 | Go2 桥接的空跑、速度限幅、输入超时和零速度行为。 | 接入前核对 `/cmd_vel`、机体坐标系、停车机制；当前不执行实机联动。 |
| 主仓库与 SCAN | 统一构建、bringup、管理器任务流和 SCAN 状态。 | 真实路径进入 `/nav3/managed_path` 后的轨迹、取消、故障与零速度。 |

出现无法快进、接口不匹配或测试失败时，先保留现场和日志，与另一方确认提交基线；不使用 `git push --force` 覆盖对方历史。
