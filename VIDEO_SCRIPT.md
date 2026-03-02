# 视频作业完整执行规划（4 分钟）

按顺序做下面每一步即可满足作业要求。每段英文台词后附中文大意。

---

## 录制前准备（不录进视频）

1. **打开 Gazebo**：先启动你的仿真环境（例如先 launch 好带无人机和场景的 Gazebo），让场景里有障碍物和 viewpoint 可见。
2. **打开终端**：在 `mission_planning_ws` 下，确认已 `source` 好 ROS2 和 workspace。
3. **可选**：提前跑通一次 `mission_scenario.py`，确保会生成 `runs/metrics.csv`，录的时候直接展示即可。
4. **录屏**：准备好录屏软件（录整个桌面或“终端 + Gazebo”分屏），**总时长控制在 4 分钟内**。

---

## 第 1 段：开场介绍（约 45 秒）

**画面：** 可以是一张简单标题页，或直接是 Gazebo 场景（能看到障碍物和无人机）。

**你要做的：** 对着镜头/麦克风说下面几句（可照读或用自己的话复述）。

**台词（英文）：**

1. *"Hi. This video shows our path-planning solution for a drone inspection task in Gazebo."*
2. *"The drone must visit several viewpoints, avoid obstacles, and at each viewpoint we use the camera to detect an ArUco marker to confirm the visit."*
3. *"We use two ideas, which are implemented in planner.py: the first is waypoint optimisation with TSP to give the best visit order, and the second is the A\* algorithm to find a collision-free path between consecutive waypoints. The full path is then sent to the node that controls the flight."*
4. *"The flight runs in Gazebo. Let me show you one full run."*

**中文大意：**  
大家好。这个视频展示我们在 Gazebo 里做的无人机巡检路径规划。无人机要访问多个观测点、避开障碍物，并在每个观测点用相机检测 ArUco 标记来确认到达。我们在 planner.py 里实现了两点：一是用 TSP 做航点优化得到最佳访问顺序，二是用 A* 在相邻航点间规划碰撞自由路径，最后把整条路径发给控制飞行的节点。接下来给大家看一次完整飞行。

---

## 第 2 段：在 Gazebo 里跑一次完整任务（约 2 分钟～2 分 15 秒）

**画面：** 终端和 Gazebo 同时可见（左右分屏或画中画都可以）。从**输入命令**到**降落结束**都要在画面里。

### 步骤 2.1 – 展示场景并启动

- **操作：** 让观众先看到 Gazebo 窗口（有障碍物、有无人机的场景）。
- **操作：** 切到终端，输入并执行（把 scenario 换成你用的，例如 scenario1 或 scenario4）：
  ```bash
  cd /home/tianhaoliang/mission_planning_ws/src/challenge_mission_planning
  python3 mission_scenario.py scenarios/scenario4.yaml
  ```
- **说一句（英文）：** *"I run the mission script. The drone will take off and follow the planned path."*  
  **中文：** 我运行任务脚本，无人机会起飞并沿规划路径飞行。

### 步骤 2.2 – 起飞

- **画面：** 无人机在 Gazebo 里起飞。
- **说一句（英文）：** *"The drone is taking off."*  
  **中文：** 无人机正在起飞。

### 步骤 2.3 – 飞行路径（重点：航点优化 + 避障）

- **画面：** 无人机沿路径飞，**当它明显绕开某个障碍物时**，或**在绕行点短暂悬停时**，指一下。
- **说一句（英文）：** *"It follows the optimised waypoints. Here you see it goes around the obstacle — that is our collision-free planning."*  
  **中文：** 它按优化后的航点飞；这里可以看到它绕开障碍物，这就是我们的碰撞自由规划。
- **（可选）若飞机在某个点停顿/悬停：**  
  **英文：** *"It pauses here because this is a detour waypoint. The drone cannot fly in a straight line to the next viewpoint because of the obstacle, so the algorithm planned this waypoint in between."*  
  **中文：** 这里停顿了一下，是因为这是一个绕行航点；飞机没法直线飞向下一个 viewpoint（被障碍物挡住了），所以我们在中间规划了这个 waypoint。

### 步骤 2.4 – 观测点与 ArUco（可选，时间紧就一句话带过）

- **画面：** 无人机在某个 viewpoint 悬停/转向时，或终端里出现 “ArUco marker … verified” 时。
- **说一句（英文）：** *"At each viewpoint it turns to look at the marker and we verify the ArUco ID."*  
  **中文：** 在每个观测点它会转向对准标记，我们通过 ArUco ID 来验证。

### 步骤 2.5 – 降落与完成证明

- **画面：** 无人机降落。
- **说一句（英文）：** *"Now it lands. The run is done."*  
  **中文：** 现在它降落了，这一趟跑完了。

- **画面：** 把终端拉近或切到终端，让观众看到最后一行类似：
  `Mission done: XXs | VPs 10/10 | ArUco 10/10 | .../runs/metrics.csv`
- **说一句（英文）：** *"See here: all viewpoints visited, all ArUco markers verified. So the mission succeeded."*  
  **中文：** 看这里：所有观测点都访问了，所有 ArUco 都验证了，任务成功完成。

---

## 第 3 段：简单性能分析（约 45 秒）

**画面：** 打开 `runs/metrics.csv`（用 Excel、VS Code 或 `cat` 在终端里显示都可以），让观众看到表头和最近一行数据。

**你要做的：** 只讲 3～4 个数字，不要展开公式或实现细节。

**台词（英文）：**

1. *"We save each run for performance analysis. I open the metrics file."*
2. *"For this run: mission time is X seconds, flight distance is Y meters."*（把 X、Y 换成你 CSV 里这一行的 execution_s 和 optimized_dist_m）
3. *"We visited all viewpoints and verified all ArUco markers, and the path was collision-free. So success is true."*
4. *"The improvement column shows that TSP waypoint optimisation reduced the distance compared to the baseline order."*（可选，如果表里有 improvement 那一列）

**中文大意：**  
我们把每次运行都记下来做性能分析。打开指标文件。这一趟：任务时间是 X 秒，飞行距离是 Y 米。我们访问了全部观测点并验证了全部 ArUco，路径是碰撞自由的，所以成功。改进率那一列说明 TSP 航点优化相比基线顺序减少了飞行距离。

---

## 第 4 段：结尾（约 15 秒）

**画面：** 可以回到 Gazebo 静止画面，或黑屏/标题页。

**台词（英文）：**  
*"So that is our solution: waypoint optimisation, collision-free motion planning, and performance analysis, all in Gazebo. Thanks for watching."*

**中文大意：**  
以上就是我们的方案：航点优化、碰撞自由运动规划和性能分析，都在 Gazebo 里完成。谢谢观看。

---

## 时间分配小结（总长 4 分钟）

| 段落 | 时间 | 内容 |
|------|------|------|
| 1. 开场 | ~0:45 | 问题 + 航点优化 + 碰撞自由 + Gazebo，不写代码 |
| 2. Gazebo 运行 | ~2:00–2:15 | 启动 → 起飞 → 飞路径（指一次绕障）→ 降落 → 展示终端/完成证明 |
| 3. 性能分析 | ~0:45 | 打开 CSV，说时间、距离、成功、碰撞自由 |
| 4. 结尾 | ~0:15 | 一句话总结 |

---

## 关于是否要讲代码（函数、文件怎么配合）

**不需要在视频里逐段讲代码。** 作业要求是：把问题、做法、演示和性能说清楚，没有要求讲“哪个函数做什么、文件和函数如何配合”。4 分钟也讲不完代码细节。

**你只需要：**
- 用**一两句话**说清整体流程即可，例如：*"We load the scenario, plan the path with TSP and detours, then the script flies the drone and checks ArUco at each viewpoint."*（我们读场景、用 TSP 和绕行规划路径，然后脚本控制飞机飞并在每个观测点检测 ArUco。）
- **不用**逐个文件、逐个函数介绍（例如不用讲 `planner.py` 里哪个函数算距离、`mission_scenario.py` 里 `drone_run` 怎么循环）。

**录屏时终端会打印：**
- 正在去哪个点：`Go to VP3 (2/10) with yaw=...` 或 `Go to detour (3/10)`，方便你指着说“现在去第几个点 / 绕行点”。
- 是否检测到 marker：`ArUco marker 14 verified at VP3` 或 `ArUco marker 14 NOT detected at VP3`，方便你指着说“这里验证到了 / 这里没检测到”。

如果老师明确要求“要讲代码结构或函数分工”，再单独录一段 1～2 分钟的代码讲解即可；否则按当前脚本只讲“做了什么 + 演示 + 指标”就够。

---

## 提交前自检

- [ ] 总时长约 4 分钟（可略短，不要超）。
- [ ] 明确说了/展示了：在 **Gazebo 仿真**里做。
- [ ] 提到了 **waypoint optimisation**（TSP/优化航点）。
- [ ] 有一次 **绕障碍**的镜头或说明（**collision-free**）。
- [ ] 展示了 **performance analysis**（时间、距离、成功、collision-free，来自 CSV 或终端）。
- [ ] 至少一次完整飞行：起飞 → 飞行 → 降落；ArUco 验证在终端或 CSV 里可见。
- [ ] 结尾一句总结：waypoint optimisation + collision-free + performance analysis in Gazebo。

按上面“打开什么、说什么、展示什么”一步步做，就能完整满足视频作业要求。祝你录制顺利。
