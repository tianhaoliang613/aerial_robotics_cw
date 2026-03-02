# 项目学习笔记

本文件记录在 Structural Inspection Path Planning Challenge 项目中学到的知识。

---

## 0. 代码架构与问题解决概要

### 0.1 代码架构

| 文件 | 职责 | 主要内容 |
|------|------|----------|
| **planner.py** | 规划核心 | 场景解析、碰撞检测、距离矩阵、TSP（exact/2-opt）、A* 与路径简化、`plan_full_route`；NN/方法对比/可视化/CLI 为薄包装，实现在 planner_tools。 |
| **planner_tools.py** | 工具与 CLI | 最近邻 TSP、`compare_planning_methods`、`write_method_comparison_csv`、`visualize_scenario`、`main()`。`python3 planner.py scenario.yaml -v` 会转调此处。 |
| **mission_scenario.py** | 任务与证明 | 起飞/降落、按 waypoint 执行（viewpoint 用 `go_to_point_with_yaw`，绕行用 `go_to_point`）、ArUco 按需订阅、写 `runs/metrics.csv` 与 JSON、控制台 MISSION REPORT。 |

数据流：mission_scenario 读场景 → planner 得到 waypoints 与 visit_order → 执行飞行与 ArUco → 写 CSV/JSON。方法对比与 3D 图在 planner_tools 中离线完成。

### 0.2 关键问题与解决

细节见 **work_log.md**。

- **TSP**：>15 节点用 2-opt 且 `max_processing_time=10`，避免爆内存或卡死。
- **A***：`round` 取整 + `_find_free_cell` 避免 goal 落障碍内；closed_set、边界、搜索上限；buffer 0.8m。
- **ArUco**：独立节点 + 独立 executor，仅到 viewpoint 时临时订阅相机，检测完即销毁，避免拖垮飞行控制。
- **仿真**：换场景前 `./stop.bash` 等清理；Goal Rejected 重试；viewpoint 后 VIEWPOINT_SETTLE_S 再发下一段。

### 0.3 规划模块 CLI

`python3 planner.py [scenario.yaml] [-v]`：无 `-v` 只打文本（场景信息、TSP 顺序与距离、waypoints 数与实际距离）；有 `-v` 另弹 3D 图（障碍、viewpoint、路线），可截图为报告用。

---

## 1. 装饰器（Decorator）

### 1.1 从基本概念开始

在 Python 里，**函数是一个对象**，可以像数字和字符串一样被传来传去：

```python
def greet():
    print("hello")

f = greet       # 函数赋值给变量
f()             # 输出: hello

def call_twice(func):
    func()
    func()

call_twice(greet)  # 输出: hello hello
```

### 1.2 装饰器就是"接收函数、返回新函数"的函数

```python
def make_louder(func):
    """接收一个函数，返回一个加强版的新函数"""
    def new_func():
        print("===开始===")
        func()
        print("===结束===")
    return new_func
```

手动使用：

```python
def greet():
    print("hello")

greet = make_louder(greet)   # 用加强版替换原来的
greet()
# 输出:
# ===开始===
# hello
# ===结束===
```

### 1.3 `@` 只是上面那行替换的简写

```python
@make_louder          # 等价于: greet = make_louder(greet)
def greet():
    print("hello")

greet()
# 输出:
# ===开始===
# hello
# ===结束===
```

**一句话：`@xxx` 的意思就是——定义完函数后，立刻执行 `函数 = xxx(函数)`。**

使用前提：`xxx` 必须已经定义或 import 过，否则报 `NameError`。

### 1.4 常见内置装饰器

| 装饰器 | 作用 | 用在哪 |
|--------|------|--------|
| `@dataclass` | 自动生成 `__init__`, `__repr__`, `__eq__` | 类 |
| `@property` | 让方法伪装成属性，调用时不用括号 | 类的方法 |
| `@staticmethod` | 不需要 `self` 的方法 | 类的方法 |
| `@classmethod` | 第一个参数是类本身（`cls`）而非实例 | 类的方法 |
| `@abstractmethod` | 强制子类必须实现该方法 | 抽象基类 |
| `@lru_cache` | 自动缓存函数返回值 | 函数 |

---

## 2. `@dataclass` —— 自动生成类的样板代码

传统写法需要手动写重复的 `__init__`：

```python
class Viewpoint:
    def __init__(self, id, x, y, z, yaw):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
```

`@dataclass` 自动生成等价代码，只需声明属性和类型：

```python
from dataclasses import dataclass

@dataclass
class Viewpoint:
    """本项目中表示无人机需要访问的观测点"""
    id: int
    x: float
    y: float
    z: float
    yaw: float   # w in the scenario file (radians)

    @property
    def position(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
```

使用方式：

```python
vp = Viewpoint(id=1, x=3.0, y=4.0, z=2.0, yaw=1.57)
print(vp.x)        # 3.0
print(vp)           # Viewpoint(id=1, x=3.0, y=4.0, z=2.0, yaw=1.57)
                    # ↑ __repr__ 自动生成，不再是没用的内存地址
print(vp.position)  # [3.0, 4.0, 2.0]  ← @property，不需要括号
```

`@dataclass` 自动生成了三个特殊方法：
- `__init__`：构造函数
- `__repr__`：`print()` 时显示有意义的内容
- `__eq__`：`==` 比较时逐属性对比

---

## 3. `@property` —— 让方法伪装成属性

本项目中 `Obstacle` 类需要从中心坐标和尺寸计算出包围盒的两个角点：

```python
@dataclass
class Obstacle:
    """轴对齐长方体障碍物"""
    id: int
    x: float       # 中心 x
    y: float       # 中心 y
    z: float       # 底面 z
    width: float   # x 方向宽度
    depth: float   # y 方向深度
    height: float  # z 方向高度

    @property
    def min_corner(self) -> np.ndarray:
        return np.array([self.x - self.width/2, self.y - self.depth/2, self.z])

    @property
    def max_corner(self) -> np.ndarray:
        return np.array([self.x + self.width/2, self.y + self.depth/2, self.z + self.height])
```

使用时像普通属性一样，不需要加括号：

```python
obs = Obstacle(id=1, x=7.15, y=1.99, z=3.68, width=1.82, depth=1.72, height=1.50)
print(obs.min_corner)  # [6.24, 1.13, 3.68]
print(obs.max_corner)  # [8.06, 2.85, 5.18]
```

如果不用 `@property`，就得写成 `obs.get_min_corner()` 加括号调用。

---

## 4. Python 特殊方法（Dunder Methods）

名字以双下划线开头和结尾（dunder = double underscore）。

| 方法 | 触发时机 | 示例 |
|------|---------|------|
| `__init__(self, ...)` | 创建对象 | `vp = Viewpoint(1, 3.0)` |
| `__repr__(self)` | `print(obj)` | `print(vp)` |
| `__eq__(self, other)` | `==` 比较 | `vp1 == vp2` |
| `__len__(self)` | `len()` | `len(my_list)` |
| `__add__(self, other)` | `+` 运算 | `obj1 + obj2` |
| `__getitem__(self, key)` | `[]` 索引 | `obj[0]` |

没有 `__repr__` 时，`print(obj)` 输出 `<__main__.Viewpoint object at 0x7f3b...>`（没用的内存地址）。`@dataclass` 自动生成了 `__repr__`，所以 `print(vp)` 会显示所有属性值。

---

## 5. YAML 文件解析

`yaml.safe_load()` 将 YAML 文件加载成 Python 嵌套字典：

```yaml
# scenario1.yaml
drone_start_pose:
  x: 0.0
  y: 0.0
viewpoint_poses:
  1:
    x: -9.51
    w: 2.01
```

加载后：

```python
import yaml

with open('scenarios/scenario1.yaml', 'r') as f:
    raw = yaml.safe_load(f)

# raw 就是一个嵌套字典:
# {
#     'drone_start_pose': {'x': 0.0, 'y': 0.0, 'z': 0.0},
#     'viewpoint_poses': {
#         1: {'x': -9.51, 'w': 2.01, ...},
#         2: {...},
#     },
#     'obstacles': {...}
# }

# .get(key, default) 安全访问（键不存在时返回默认值而非报错）
raw.get('viewpoint_poses', {})

# .items() 遍历键值对
for vid, vp in raw['viewpoint_poses'].items():
    print(vid, vp['x'])  # 1, -9.51
```

---

## 6. 组合优化与 TSP

### 6.1 组合优化（Combinatorial Optimization）

从有限个离散方案中找到最优解的一类问题。例如：10 个 viewpoint 的访问顺序有 `10! = 3,628,800` 种排列，从中找距离最短的那个。

### 6.2 TSP（旅行商问题）

组合优化中最经典的模型：给定一组城市和两两之间的距离，找一条最短的回路访问所有城市。本项目的 viewpoint 访问顺序就是一个 TSP 问题。

### 6.3 常见求解方法

| 方法 | 思路 | 特点 |
|------|------|------|
| TSP 精确求解 | 穷举/分支定界找全局最优 | 点少时最优，点多算不动 |
| TSP 启发式（最近邻、2-opt） | 贪心构造 + 局部改进 | 快速，接近最优 |
| 遗传算法（GA） | 模拟进化，随机变异 + 选择 | 通用性强 |
| 蚁群算法（ACO） | 模拟蚂蚁信息素寻路 | 适合图上路径优化 |
| 模拟退火（SA） | 允许暂时接受差解来跳出局部最优 | 简单易实现 |

本项目使用 `python_tsp` 库，对 5-20 个点的规模，TSP 是最直接的选择。

---

## 7. 路径规划算法对比：A* vs RRT vs PRM

### 7.1 三种算法的核心思路

**A*（A-Star）**：在网格上系统搜索。把空间切成小格子，从起点开始，每次向评分最优的相邻格子扩展，直到到达终点。评分 `f = g + h`，其中 `g` 是已走距离，`h` 是到终点的直线距离估计。

**RRT（Rapidly-exploring Random Tree）**：随机采样生长树。每一步在空间中随机取一个点，从树上已有的最近节点向它延伸一小段，如果不碰障碍物就加入树中。重复直到树长到终点附近。

**PRM（Probabilistic Roadmap）**：预处理建图。在空间中随机撒大量点，把互相可以直线相连（不碰障碍物）的点连成一张图，然后在这张图上用 Dijkstra 搜最短路。适合同一环境下多次查询不同起终点。

### 7.2 对比

| | A* | RRT | PRM |
|------|------|------|------|
| 原理 | 网格搜索 | 随机树生长 | 预采样建图 |
| 路径最优性 | 最优（给定分辨率下） | 不保证最优 | 不保证最优 |
| 确定性 | 确定（每次结果一样） | 随机（每次不同） | 随机 |
| 低维空间（2D/3D） | 非常快 | 快 | 快（需预处理） |
| 高维空间（6D+） | 网格爆炸，不可用 | 快，核心优势 | 快 |
| 适合场景 | 小空间、简单障碍物 | 高维、复杂环境 | 同一环境多次查询 |

### 7.3 什么是"高维空间"

维度 = 描述状态需要的数字个数。

- 无人机位置：`(x, y, z)` → 3 维
- 6 关节机械臂：`(θ1, θ2, θ3, θ4, θ5, θ6)` → 6 维

A* 需要建网格，网格数量随维度指数增长：

| 维度 | 每轴 100 格时的总格子数 |
|------|------|
| 2D | 100² = 10,000 |
| 3D | 100³ = 1,000,000 |
| 6D | 100⁶ = 1,000,000,000,000 |

6 维就要 1 万亿个格子，A* 跑不动。RRT 不建网格，每步只是采样一个点 + 一次碰撞检测，所以高维空间下效率远高于 A*。

### 7.4 本项目的选择

选 A*，理由：
- 空间只有 3 维，范围 ±10m，分辨率 0.5m，网格 ~16000 格，A* 轻松处理
- 障碍物少（1-10 个）且是简单几何体（轴对齐长方体）
- 路径最优 + 确定性，方便调试和写报告
- 题目只要求实现一种路径规划并说明理由

---

## 8. 队列：普通队列 vs 优先队列

### 8.1 普通队列（deque）：先进先出

像排队买奶茶——谁先来谁先服务。

```python
from collections import deque

queue = deque()
queue.append("小明")      # 小明先来
queue.append("小红")      # 小红第二个
queue.append("小张")      # 小张第三个

queue.popleft()           # 小明（先进先出）
queue.popleft()           # 小红
queue.popleft()           # 小张
```

用在 BFS（广度优先搜索），一层一层均匀扩展。

### 8.2 优先队列（heapq）：最小值先出

像医院急诊——不管谁先来，病情最重（值最小）的先看。

```python
import heapq

queue = []
heapq.heappush(queue, (5.0, "小明"))    # f=5.0
heapq.heappush(queue, (2.0, "小红"))    # f=2.0
heapq.heappush(queue, (8.0, "小张"))    # f=8.0

heapq.heappop(queue)   # (2.0, "小红")  ← f 最小的先出
heapq.heappop(queue)   # (5.0, "小明")
heapq.heappop(queue)   # (8.0, "小张")
```

存的是元组 `(f值, 数据)`，Python 比较元组先比第一个元素，所以 f 值最小的先弹出。用在 A* 和 Dijkstra。

### 8.3 对比

| | 普通队列 deque | 优先队列 heapq |
|---|---|---|
| 弹出规则 | 先进先出 | 最小值先出 |
| 插入 | `append()` | `heappush()` |
| 弹出 | `popleft()` | `heappop()` |
| 用途 | BFS | A*, Dijkstra |

---

## 9. A* 算法完整流程

### 9.1 核心思想

在网格上搜索从起点到终点的最短路径。每个格子有评分 `f = g + h`：
- `g`：从起点到当前格子已经走了多远（确切值）
- `h`：从当前格子到终点估计还要走多远（直线距离）
- `f`：总预估路程，A* 每一步选 f 最小的格子来探索

### 9.2 完整流程示例

2D 网格，S 是起点，G 是终点，# 是障碍物：

```
S . . .
. . # .
. . # G
```

**初始化：**

```python
open_set = [(0.0, S)]        # 待探索队列，起点 f=0
g_score = {S: 0.0}           # 从起点到各格子的最短距离，只有起点=0
came_from = {}               # 每个格子从哪来的
```

g_score 初始只有一个条目：起点到起点的距离是 0。后续每探索一个格子，就把它邻居的 g 值写入字典，像滚雪球一样越来越大。

**第 1 步：** 弹出 S(0,0)，此时 `g_score[S] = 0.0`。看邻居 (1,0) 和 (0,1)：

- (1,0)：`tentative_g = g_score[S] + 1 = 0 + 1 = 1`，h=2.8，f=3.8
- (0,1)：`tentative_g = g_score[S] + 1 = 0 + 1 = 1`，h=3.2，f=4.2

两个邻居之前都不在 g_score 里（当作无穷大），1 < 无穷大成立，所以写入：

```python
g_score   = { S: 0.0, (1,0): 1.0, (0,1): 1.0 }
came_from = { (1,0): S, (0,1): S }
open_set  = [(3.8, (1,0)), (4.2, (0,1))]
```

**第 2 步：** 弹出 f 最小的 (1,0)，此时 `g_score[(1,0)] = 1.0`（上一步写入的）。看邻居 (2,0) 和 (1,1)：

- (2,0)：`tentative_g = g_score[(1,0)] + 1 = 1 + 1 = 2`
- (1,1)：`tentative_g = g_score[(1,0)] + 1.414 = 1 + 1.414 = 2.414`（对角线）

```python
g_score   = { S: 0, (1,0): 1, (0,1): 1, (2,0): 2, (1,1): 2.414 }
came_from = { (1,0): S, (0,1): S, (2,0): (1,0), (1,1): (1,0) }
```

**第 3 步：** 弹出 (0,1)，此时 `g_score[(0,1)] = 1.0`（第 1 步写入的）。看邻居 (0,2)：

- (0,2)：`tentative_g = g_score[(0,1)] + 1 = 1 + 1 = 2`

注意 (1,1) 也是 (0,1) 的邻居：`tentative_g = 1 + 1.414 = 2.414`，和已有的 g=2.414 一样，不更短，跳过。

```python
g_score   = { S: 0, (1,0): 1, (0,1): 1, (2,0): 2, (1,1): 2.414, (0,2): 2 }
came_from = { (1,0): S, (0,1): S, (2,0): (1,0), (1,1): (1,0), (0,2): (0,1) }
```

**要点：** 每一轮循环中 `g_score[current]` 的值都是之前某一轮写入的。起点 g=0 是种子，每次从 current 出发，把 `g_score[current] + 移动代价` 写入邻居。

**后续步骤：** 继续搜索，从右上方绕过障碍物，最终到达 G(3,2)。

```python
came_from = {
    (1,0): (0,0),   (0,1): (0,0),
    (2,0): (1,0),   (1,1): (1,0),
    (3,0): (2,0),   (0,2): (0,1),
    (3,1): (3,0),
    (3,2): (3,1),   # 终点
}
```

**回溯路径：** 从终点沿 came_from 往回找：

```
G(3,2) → 查 came_from[(3,2)] = (3,1)
(3,1)  → 查 came_from[(3,1)] = (3,0)
(3,0)  → 查 came_from[(3,0)] = (2,0)
(2,0)  → 查 came_from[(2,0)] = (1,0)
(1,0)  → 查 came_from[(1,0)] = S(0,0)
S(0,0) → 不在 came_from 里，停止
```

反转得到路径：`S → (1,0) → (2,0) → (3,0) → (3,1) → G`

### 9.3 为什么用 came_from 字典而不是列表记录路径

A* 的探索顺序是 `S, (1,0), (0,1), (2,0), (0,2), ...`——这不是一条路径，而是探索的先后顺序。其中 (1,0) 和 (0,1) 属于两条不同方向的探索。

列表记录的是"我去过哪些格子"，但没有"谁到谁"的连接关系。`came_from` 字典记录的是每个格子是从哪个格子来的，所以能从终点回溯出一条真正连续的路径。

### 9.4 A* 核心代码中"探索邻居"的逻辑

```python
for offset in neighbors_offsets:          # 遍历 26 个方向
    neighbor = current + offset           # 邻居坐标
    neighbor_pos = to_world(neighbor)     # 转成真实坐标

    # 1. 邻居在障碍物里？跳过
    if is_point_in_obstacle(neighbor_pos, obstacles):
        continue

    # 2. 算移动代价（直线0.5m, 对角0.707m, 体对角0.866m）
    move_cost = sqrt(dx² + dy² + dz²) * resolution

    # 3. 经过 current 到达 neighbor 的总距离
    tentative_g = g_score[current] + move_cost

    # 4. 比 neighbor 已知的最短距离更短吗？
    #    如果 neighbor 从没探索过，g_score.get 返回无穷大，必然更短
    if tentative_g < g_score.get(neighbor, float('inf')):
        g_score[neighbor] = tentative_g         # 更新最短距离
        f = tentative_g + heuristic(...)        # f = g + h
        came_from[neighbor] = current           # 记录来源
        heapq.heappush(open_set, (f, neighbor)) # 加入优先队列
```

第 4 步的条件判断是关键：同一个邻居可能被多条路径发现。比如格子 X 可以从 A 走过来（g=5），也可以从 B 走过来（g=3）。当从 B 发现 X 时，tentative_g=3 < 已有的 g=5，所以更新为更短的路径。

### 9.5 came_from 与 g_score 的动态覆盖示例（重点）

用一个简化图说明为什么 `came_from` 可以被覆盖，以及如何保证“起点到 B 的路径”也被同步修正：

```
S - A - B
 \      /
   C ---
```

边权：
- `S->A = 2`
- `A->B = 2`（总长 4）
- `S->C = 1`
- `C->B = 1`（总长 2，更短）

#### 初始化

```python
g_score   = {S: 0}
came_from = {}
open_set  = [(0, S)]
```

#### 步骤1：从 S 扩展 A、C

```python
g_score   = {S:0, A:2, C:1}
came_from = {A:S, C:S}
```

#### 步骤2：先通过 A 发现 B

```python
tentative_g(B via A) = g_score[A] + 2 = 4
g_score[B] = 4
came_from[B] = A
```

当前“起点到 B”路径是：`S -> A -> B`

#### 步骤3：后来通过 C 再次发现 B（更短）

```python
tentative_g(B via C) = g_score[C] + 1 = 2
# 2 < 当前 g_score[B]=4，触发覆盖
g_score[B] = 2
came_from[B] = C
```

现在“起点到 B”路径自动变成：`S -> C -> B`

#### 结论

- `came_from[child] = parent` 记录的是“当前最优前驱”，不是第一次发现时的前驱。
- `g_score` 变短时，`came_from` 会被同步覆盖。
- 最终从终点回溯时，得到的是覆盖后的最短路径链。

---

## 10. 仿真与运行命令速查（实践）

本节用于记录本项目常用命令，便于复现实验与写报告。命令来源：项目 `README.md` 的 `Execution` 章节（并结合当前 `mission_scenario.py` 实际用法）。

### 10.1 每次全新启动（推荐）

终端 1（启动 Gazebo + AS2）：

```bash
source /opt/ros/humble/setup.bash
source /home/tianhaoliang/mission_planning_ws/install/setup.bash
cd /home/tianhaoliang/mission_planning_ws/src/challenge_mission_planning
./launch_as2.bash -s scenarios/scenario1.yaml
```

终端 2（可选，启动地面站/RViz）：

```bash
source /opt/ros/humble/setup.bash
source /home/tianhaoliang/mission_planning_ws/install/setup.bash
cd /home/tianhaoliang/mission_planning_ws/src/challenge_mission_planning
./launch_ground_station.bash
```

终端 3（执行任务脚本）：

```bash
source /opt/ros/humble/setup.bash
source /home/tianhaoliang/mission_planning_ws/install/setup.bash
cd /home/tianhaoliang/mission_planning_ws/src/challenge_mission_planning
python3 mission_scenario.py scenarios/scenario1.yaml
```

### 10.2 换场景运行（scenario1~4）

先停旧仿真：

```bash
cd /home/tianhaoliang/mission_planning_ws/src/challenge_mission_planning
./stop.bash
```

如仍有残留，再清理（可选）：

```bash
tmux kill-server || true
pkill -9 -f gzserver || true
pkill -9 -f gzclient || true
pkill -9 -f "ros2" || true
```

然后重新按 10.1 启动，只把场景改成：

```bash
./launch_as2.bash -s scenarios/scenario2.yaml
python3 mission_scenario.py scenarios/scenario2.yaml
```

同理可换成 `scenario3.yaml`、`scenario4.yaml`。

### 10.3 说明：RViz 不是必需

- 任务执行核心是终端 1 + 终端 3
- 终端 2（RViz）主要用于可视化；若崩溃，不影响任务脚本执行
- 若出现 `TakeoffBehavior/GoToBehavior Not Available`，通常是仿真节点未完全就绪，重启一次 10.1 流程最稳

### 10.4 ArUco 一直显示 NOT detected 的含义

`ArUco marker XX NOT detected at VPY` 的意思是：无人机到达该 viewpoint 后，在检测窗口内没有在相机画面中识别到期望的 marker id。

常见原因：
1. 该 viewpoint 的相机朝向/位置不足以看到 marker（角度偏了、距离太远、遮挡）
2. 运动中图像模糊（速度快，停留时间短）
3. 检测依赖异常（本机 `numpy/opencv` ABI 不兼容时，检测模块不可用）

当检测模块不可用时，日志也会持续出现 `NOT detected`，但这不一定代表路径访问失败，通常表示“看到了 viewpoint，但没有得到可靠视觉验证”。

### 10.5 结束仿真

```bash
cd /home/tianhaoliang/mission_planning_ws/src/challenge_mission_planning
./stop.bash
```

---

## 11. 日志记录与指标

用于**验证完成**、**比较方法**、**向客户证明**。单次任务在 **mission_scenario.py** 的 `__main__` 中、`drone_run` 之后一次性写；方法对比在 **planner_tools.py**（`compare_planning_methods`、`write_method_comparison_csv`），planner 仅薄包装。

### 11.1 输出一览

| 输出 | 位置 | 作用 |
|------|------|------|
| 控制台 MISSION REPORT | mission_scenario.py | 本次完成情况、无碰撞、速度 |
| runs/metrics.csv | 同上，DictWriter 追加 | 多 run 汇总；含 \texttt{order\_type}（\texttt{tsp} \| \texttt{baseline}） |
| runs/<ts>_<scenario>.json | 同上，json.dump | 单次证据：visit_order、visited、aruco_verified、aruco_detected_ids |
| runs/method_comparison.csv | planner_tools | Baseline/TSP/NN 规划距离与时间 |

**Baseline 实际飞行时间**：用 \texttt{--baseline} 按 YAML 顺序飞一遍，可得到该场景下 baseline 的 \texttt{execution\_s}，与 TSP 的飞行时间对比（距离短不一定飞得短）。每场景单独跑一次 \texttt{python3 mission\_scenario.py scenarios/<scenario>.yaml --baseline}，从 CSV 或 JSON 取 \texttt{execution\_s} 填入报告 Table 1 的 ``Execution time [Baseline]'' 行。

### 11.2 实现要点

- **指标计算**：`drone_run` 返回后算 `exec_time`、`collision_free`（逐段 `is_path_clear`）、`avg_speed`；`aruco_state['visited'/'verified']` 来自执行过程。
- **CSV**：`runs/metrics.csv` 追加一行，字段含 timestamp、scenario、planning_s、execution_s、viewpoints_visited、aruco_verified、optimized_dist_m、baseline_dist_m、improvement_pct、avg_speed_mps、collision_free。
- **JSON**：`runs/<timestamp>_<scenario>.json` 含 summary + visit_order + visited + aruco_verified + aruco_detected_ids，便于逐项核对。
- **方法对比**：`compare_planning_methods(scenario_paths)` 对每场景算 Baseline/TSP/NN 的规划距离与耗时；`write_method_comparison_csv` 写入 CSV，报告中的方法表可由此生成。

---

## 12. 报告中的图片

报告（`report.tex`）中 4 张图：Figure 1 为 Gazebo 截图，Figure 2 为规划路径 3D 图。`\includegraphics` 的路径以编译时工作目录为准（与 report.tex 相对位置一致即可）。

### 12.1 清单与获取

| 图 | 内容 | 获取方式 |
|----|------|----------|
| scenario1_gazebo.png | 场景 1 仿真界面（10 VP, 1 障碍） | 启动 scenario1 后 Gazebo 截图 |
| scenario4_gazebo.png | 场景 4 仿真界面（5 VP, 10 障碍） | 启动 scenario4 后 Gazebo 截图 |
| path_scenario1.png | 场景 1 规划路径 3D | `python3 planner.py scenarios/scenario1.yaml -v`，窗口保存 |
| path_scenario4.png | 场景 4 规划路径 3D（含 A* 绕行） | `python3 planner.py scenarios/scenario4.yaml -v`，窗口保存 |

路径图由 planner_tools 的 `visualize_scenario` 生成，无需开 Gazebo。
