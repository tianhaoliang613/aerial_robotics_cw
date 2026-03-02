# 工作记录

当前代码架构与文件分工见 **learning_notes.md §0**。本文仅记录开发步骤与遇到的问题与解决。

## 大纲

| 步骤 | 内容 | 状态 |
|------|------|------|
| Step 1 | 场景数据解析 + 3D 障碍物表示 + 碰撞检测 | 已完成 |
| Step 2 | TSP 求解最优访问顺序 | 已完成 |
| Step 3 | 路径规划（A*）- 两点间无碰撞路径 | 已完成 |
| Step 4 | 整合进任务脚本 - 无人机按优化路径飞行 | 已完成 |
| Step 5 | 启动仿真测试 - Gazebo 中验证飞行效果 | 已完成 |
| Step 6 | ArUco 检测验证 - 到达 viewpoint 后确认拍到标记 | 已完成 |
| Step 7 | 指标记录与分析 - 飞行时间、距离、速度、成功率 | 已完成 |
| Step 8 | 报告与视频 | 进行中 |

---

## Step 1：场景解析 + 碰撞检测

### 做了什么

在 **`planner.py`** 中实现了核心逻辑，在 **`planner_tools.py`** 中实现工具与可视化：

1. **数据结构**（planner.py）：用 `@dataclass` 定义了 `Viewpoint`、`Obstacle`、`Scenario` 三个类，将 YAML 原始数据结构化
2. **场景解析**（planner.py）：`load_scenario()` 读取 YAML 文件，用 `yaml.safe_load()` 加载为嵌套字典，再转为上述数据结构
3. **碰撞检测**（planner.py）：`segment_intersects_aabb()` 用 Slab Method 判断线段是否穿过障碍物的 AABB 包围盒
4. **距离矩阵**（planner.py）：`compute_distance_matrices()` 一次性计算欧氏距离矩阵和碰撞感知距离矩阵
5. **3D 可视化**（planner_tools.py）：`visualize_scenario()` 用 matplotlib 绘制障碍物、viewpoint 和飞行路线；`planner.py` 中保留薄包装以便统一入口

### 各场景碰撞分析

| 场景 | VP数 | 障碍物数 | 总边数 | 被阻断边 | 阻断率 |
|------|------|----------|--------|----------|--------|
| scenario1 | 10 | 1 | 45 | 1 | 2% |
| scenario2 | 10 | 5 | 45 | 3 | 7% |
| scenario3 | 20 | 1 | 190 | 22 | 12% |
| scenario4 | 5 | 10 | 10 | 7 | 70% |

### 设计决策

- **碰撞感知距离矩阵**中，被障碍物阻断的边距离乘以 `penalty_factor=2.0`。这是一个粗略估计，等 Step 3 实现路径规划后会用真实绕行距离替代。
- **两个距离矩阵合并为一个函数** `compute_distance_matrices()`，一次遍历同时返回欧氏和碰撞感知两个矩阵，避免重复计算。这是根据实际使用反馈做的改进。

---

## Step 2：TSP 求解

### 做了什么

在 `planner.py` 中实现了 `solve_tsp()` 函数，调用 `python_tsp` 库求解旅行商问题。

最终版本非常简洁：
- ≤15 个节点：`solve_tsp_dynamic_programming()`（精确求解，库函数）
- \>15 个节点：`solve_tsp_local_search()` + 2-opt + 限时 10 秒（启发式，库函数）

### TSP 优化效果

| 场景 | VP数 | 方法 | TSP距离 |
|------|------|------|---------|
| scenario1 | 10 | exact | 70.57m |
| scenario2 | 10 | exact | 73.62m |
| scenario3 | 20 | 2-opt | 89.47m |
| scenario4 | 5 | exact | 79.24m |

### 遇到的问题

**问题 1：精确求解对 21 个节点爆内存**

`solve_tsp_dynamic_programming` 的时间复杂度是 O(n² × 2ⁿ)。scenario3 有 20 个 viewpoint + 1 个起点 = 21 个节点，2²¹ ≈ 200 万个状态，直接吃掉 12GB 内存卡死。

→ 解决：对 >15 个节点自动切换为启发式方法。

**问题 2：库的启发式函数也卡死**

先后尝试了 `solve_tsp_simulated_annealing` 和 `solve_tsp_local_search`，都不设参数直接调用，全部卡死。

→ 错误判断：以为库函数对 21 个节点都太慢，于是自己写了最近邻启发式（`_solve_tsp_nearest_neighbor`），又搞了 nn+2opt 组合方案，代码越写越复杂。

→ 真正原因：没看文档。`solve_tsp_local_search` 的 `max_processing_time` 参数默认为 `None`（无限制），导致无限运行。

→ 最终解决：加上 `max_processing_time=10`，同一个函数对同样的数据不到 1 秒出结果。删掉所有自定义代码，回归简洁。

**问题 3：测试代码写死了 method="exact"**

给 `solve_tsp` 加了 `method="auto"` 自动选择机制后，底部测试代码里还是 `solve_tsp(..., method="exact")`，导致 scenario3 仍然调用精确求解而卡死。

→ 教训：改了函数逻辑后要同步更新所有调用处，先测试再提交结果。

### 教训总结

1. **用库函数前先看文档和参数默认值**，一个 `max_processing_time=None` 就导致多绕了一大圈
2. **先用最简单的方案跑通**，别一上来就搞复杂的组合方案
3. **改了函数要同步更新测试代码**，否则前面的测试结果都是无效的

---

## Step 3：路径规划（A*）

### 做了什么

在 `planner.py` 中实现了 3 个函数：

1. **`plan_path(start, goal, obstacles)`**：核心 A* 算法。先检查直线是否通畅，通的话直飞；不通才在 3D 网格上用 A* 搜索绕行路径
2. **`_simplify_path(path, obstacles)`**：将 A* 产生的锯齿路径简化，去掉不必要的中间点
3. **`plan_full_route(scenario, visit_order)`**：将 TSP 的访问顺序逐段连起来，生成完整飞行路径

### 测试结果

| 场景 | VP数 | 障碍物 | 需绕行段 | waypoints | 规划时间 |
|------|------|--------|---------|-----------|---------|
| scenario1 | 10 | 1 | 0 | 11 | 0.03s |
| scenario2 | 10 | 5 | 0 | 11 | 0.03s |
| scenario3 | 20 | 1 | 0 | 21 | 0.01s |
| scenario4 | 5 | 10 | 3 | 10-12 | 0.28s |

### 遇到的问题

**问题 1：场景 4 A* 规划卡住 60+ 秒**

场景 4 有 10 个障碍物密布。A* 搜索在 VP1→VP2 段卡住不出结果。排查发现三个 bug 叠加：

1. **Goal cell 落入障碍物**：`int()` 截断坐标时，VP2 的网格坐标映射到了障碍物内部。例如 VP2 = [-3.025, 2.168, 4.960]，`int(7.975) = 7` 映射到 [-4, 2, 4]，刚好在障碍物 4 内。A* 永远到不了终点，搜遍整个网格才放弃。
2. **没有 closed set**：已展开的节点被重复展开，浪费大量时间。
3. **没有网格边界检查**：搜索可以扩展到网格外，进一步膨胀搜索空间。

→ 解决：`int()` 改为 `round()` 减少映射误差；加 `_find_free_cell()` 保护——如果 start/goal cell 落在障碍物内，自动搜索最近的空闲格子；加入 `closed_set` 避免重复展开；加入网格边界检查和 100,000 节点搜索上限。

→ 效果：规划时间从 60+ 秒降到 **0.28 秒**。

**问题 2：无人机在障碍物群中卡住**

规划速度修复后，无人机在场景 4 的 VP1→VP2 段仍卡在障碍物前。排查发现 A* 的 buffer=0.3m 太小，生成的绕行路点距障碍物仅 0.45m，无人机控制器的位置误差和惯性导致无法安全通过。

→ 解决：buffer 从 0.3m 增大到 **0.8m**，确保路径距障碍物至少 1.07m。

→ 效果：场景 4 完美通过（5/5 VP，5/5 ArUco 验证）。

### 设计决策

- 选 A* 而非 RRT：场景空间小（±10m），障碍物少（1-10 个），A* 的确定性和路径质量更适合
- 网格分辨率 1.0m：兼顾搜索速度（格子少）和路径精度（1m 足够无人机导航）
- 安全缓冲 0.8m：确保路径在密集障碍物环境中有足够安全裕度
- 路径简化：A* 产生的网格路径有很多不必要的中间点，简化后只保留拐点

---

## Step 4：整合进任务脚本

### 在初始版本上的增量改动

保持不变：
- 版权声明头部
- `drone_start()` 原逻辑
- `drone_end()` 原逻辑
- 参数解析（`argparse`）主结构
- 主流程的启动/结束框架（`rclpy.init()`、`shutdown()`）

新增内容（增量添加）：
1. **规划预计算**（起飞前）
   - 调用 `planner.py` 的 `compute_distance_matrices`、`solve_tsp`、`plan_full_route`
   - 输出 `visit_order`、优化后距离、baseline 距离、改进百分比
2. **执行阶段改造**
   - `drone_run()` 不再按 YAML 原始顺序直飞，而是执行规划后的 waypoint 序列
   - 到 viewpoint 用 `go_to_point_with_yaw`，中间绕行点用 `go_to_point`
3. **ArUco 完成验证**
   - 独立 ROS2 节点 + 按需订阅相机话题检测 marker id
   - 在 viewpoint 停留后统计"访问数/验证数"
4. **指标与落盘**
   - 记录 planning/execution 时间、距离、平均速度、无碰撞检查结果
   - 输出到 `runs/metrics.csv` 和 `runs/<timestamp>_<scenario>.json`

---

## 仿真启动与运行命令

### 1) 启动无人机仿真节点

```bash
cd /home/tianhaoliang/mission_planning_ws/src/challenge_mission_planning
./launch_as2.bash -s scenarios/scenario1.yaml
```

> 可把 `scenario1.yaml` 改成 2/3/4。

### 2) 启动地面站（可选）

```bash
cd /home/tianhaoliang/mission_planning_ws/src/challenge_mission_planning
./launch_ground_station.bash
```

### 3) 运行任务脚本

```bash
cd /home/tianhaoliang/mission_planning_ws/src/challenge_mission_planning
python3 mission_scenario.py scenarios/scenario1.yaml
```

### 4) 换场景

```bash
./stop.bash
tmux kill-server || true
pkill -9 -f gzserver || true
pkill -9 -f gzclient || true
pkill -9 -f "ros2" || true
# 然后重新 launch_as2.bash + mission_scenario.py
```

### 5) 结束仿真

```bash
./stop.bash
```

---

## Step 5-6-7：仿真验证、ArUco 检测、指标记录

### 遇到的问题与解决方案（按时间顺序）

**问题 1：NumPy ABI 不兼容（`_ARRAY_API not found`）**

系统安装了 numpy 2.x，但 ROS2/astropy 等包是用 numpy 1.x 编译的，导致 import 时崩溃。

→ 解决：全局降级 `pip install --user "numpy<2"` 和 `python-tsp==0.4.0`。

**问题 2：仿真启动失败（`TakeoffBehavior Not Available` / `Goal Rejected`）**

Gazebo 和 ROS2 节点未完全启动就执行任务脚本，导致 action server 不可用。tmux 嵌套和残留进程也会导致冲突。

→ 解决：严格按先后顺序启动。换场景时彻底清理（`./stop.bash` + `tmux kill-server` + `pkill`）。

**问题 3：ArUco 检测模块影响飞行稳定性（核心问题，反复迭代 4 个方案）**

在最初版本代码（不含 ArUco）能完美飞完所有标记的情况下，加入 ArUco 检测后无人机开始出现各种飞行异常（从第 7 个 VP 开始失控、乱飞、不降落等）。

| 方案 | 思路 | 结果 |
|------|------|------|
| 持续订阅 + active flag | 后台持续接收帧，按需处理 | 长场景 DDS 开销积累，控制器退化 |
| 动态订阅（DroneInterface executor） | VP 到达后临时订阅 | 图像反序列化阻塞 go_to feedback |
| 独立节点 + 持续订阅 | 单独 executor 线程 | 长场景 DDS 仍积累 |
| **独立节点 + 按需订阅** | 独立节点/executor/线程，仅 VP 停留时临时 create/destroy 订阅 | **全部通过** |

→ 关键洞察：DroneInterface executor 是单线程的，高带宽相机话题（30fps, ~6MB/帧）的订阅会在 executor 上触发反序列化，与 go_to 的 action feedback 争夺执行时间。必须将相机订阅完全隔离到独立线程的独立 executor 上，且仅在需要时才订阅。

**问题 4：PX4 控制器在大 yaw 变化时走弧线**

场景 3 中，无人机偶尔出现"绕小圈"而非直线飞行。`go_to_point_with_yaw` 同时调整位置和朝向，当 yaw 变化大时 PX4 会走弧线，这是正常行为。

**问题 5：场景 2 去第八个（VP3）时跑远再回来**

VP8→VP3 段 yaw 变化约 61°，且 viewpoint 后仅等 0.5s 就发下一 goal，机体未完全稳定时 PX4 会先拐出一段再绕回，看起来像"跑远又回来"。另场景 2 偶发 Goal Rejected（重试后成功）。

→ 解决：重试前等待由 1s 改为 1.5s；新增 **VIEWPOINT_SETTLE_S = 1.0s**，在 viewpoint（ArUco 检测）后再多等 1s 再发下一段，仅对 viewpoint 生效。最新一次场景 2 运行 99.3s、无 Goal Rejected、无跑远。

---

## 最终实验结果汇总

| 指标 | Scenario 1 | Scenario 2 | Scenario 3 | Scenario 4 |
|------|-----------|-----------|-----------|-----------|
| VP 数 / 障碍物数 | 10 / 1 | 10 / 5 | 20 / 1 | 5 / 10 |
| 规划时间 | 0.02s | 0.99s | 0.09s | 0.85s |
| 执行时间 | 81.72s | 99.34s | 128.85s | 73.01s |
| 访问完成 | 10/10 | 10/10 | 20/20 | 5/5 |
| ArUco 验证 | 10/10 | 10/10 | 20/20 | 5/5 |
| 优化距离 | 60.50m | 64.61m | 82.66m | 55.19m |
| Baseline 距离 | 125.93m | 127.40m | 175.80m | 54.09m |
| 距离改进 | **52.0%** | **49.3%** | **53.0%** | -2.0% |
| 平均速度 | 0.74 m/s | 0.65 m/s | 0.64 m/s | 0.76 m/s |
| 无碰撞 | ✓ | ✓ | ✓ | ✓ |
