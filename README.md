# multi-drone-planning
I built an intelligent path planning and conflict avoidance Agent for multi-UAV cooperative missions, which solves the core pain points of time-consuming path planning, difficulty in real-time dynamic conflict avoidance, and low task scheduling efficiency in multi-UAV cooperative missions.
# 多无人机协同路径规划与冲突规避系统
Multi-Drone Cooperative Path Planning and Collision Avoidance System

基于 A* 路径规划算法与动态冲突检测机制，实现多无人机协同飞行、自动避障与终点抵达的仿真系统，支持自定义无人机数量、起点终点及障碍物，可视化直观展示飞行与避障过程。

##  功能特点
- 全局路径规划：采用 A* 算法，快速生成最短路径，同时避开预设障碍物
- 实时冲突检测：基于欧几里得距离，判定无人机间安全距离，及时识别碰撞风险
- 动态冲突规避：通过优先级调度与速度调整策略，实现无碰撞、无死锁协同飞行
- 可视化仿真：2D 动态动画，直观展示无人机飞行轨迹、避障过程及终点抵达状态
- 高度可定制：支持自定义无人机数量、起点/终点坐标、障碍物位置、飞行速度及安全距离

## 环境依赖
Python 3.7+（兼容3.7-3.11，主流版本均可）
依赖库：numpy、matplotlib（轻量无复杂依赖）

## 安装步骤
1. 克隆本仓库到本地
```bash
git clone https://github.com/你的GitHub用户名/multi-drone-planning.git
```
2. 进入项目目录
```bash
cd multi-drone-planning
```
3. 安装依赖（一行命令）
```bash
pip install -r requirements.txt
```

## 运行方式
直接执行主程序，无需额外配置，运行后自动弹出仿真窗口
```bash
python main.py
```

## 自定义配置（无需改核心代码）
打开 `main.py`，修改顶部配置参数即可实现自定义，示例如下：
```python
# 地图大小
MAP_SIZE = 30
# 无人机安全距离（小于此距离判定为冲突）
SAFE_DISTANCE = 2.0
# 每帧移动步长（控制飞行速度）
STEP_SIZE = 0.8

# 无人机设置：[起点x, 起点y, 终点x, 终点y]，可新增/删除无人机
DRONES = [
    [2, 2, 27, 27],   # 无人机1
    [2, 27, 27, 2],   # 无人机2
    [15, 2, 15, 27],  # 无人机3
]

# 障碍物设置：[x, y]，可新增/删除障碍物
OBSTACLES = [
    [10, 10], [10, 15], [10, 20],
    [15, 10], [20, 10], [20, 20]
]
```

## 项目结构
```
multi-drone-planning/
├─ main.py           # 主程序（路径规划+冲突检测+仿真可视化）
├─ README.md         # 项目说明（功能、安装、运行、配置）
├─ requirements.txt  # 依赖列表（明确版本兼容）
├─ .gitignore        # Git忽略文件（避免上传垃圾文件）
└─ LICENSE           # 许可证文件（MIT协议，开源合规）
```

## 核心技术亮点
1. A* 路径规划：兼顾路径最短与障碍物规避，运算高效，适合多无人机场景
2. 动态冲突规避：无需复杂通信协议，通过本地状态调度，实现实时避障
3. 轻量化仿真：基于matplotlib，无需专业仿真工具，开箱即用
4. 可扩展性强：可轻松扩展3D可视化、其他路径规划算法（如RRT*）、多机任务分配等功能

## 运行效果
运行后将弹出2D仿真窗口，可观察到：
- 多无人机同时从各自起点起飞，沿规划路径飞行
- 遇到障碍物自动绕行，无人机间距离过近时自动减速避让
- 所有无人机最终安全抵达各自终点，无碰撞、无死锁

## 注意事项
1. 确保Python版本为3.7+，避免依赖兼容问题
2. 若运行时未弹出仿真窗口，可检查matplotlib安装是否完整
3. 自定义无人机/障碍物数量时，建议控制在10架以内，确保仿真流畅

## 后续可扩展方向
- 新增3D可视化（基于mplot3d或PyVista）
- 集成RRT*/Dijkstra等其他路径规划算法，实现算法对比
- 增加多机任务分配功能，支持协同巡检、物资运输等场景
- 加入传感器噪声模拟，提升仿真真实性

## 许可证
本项目采用 MIT License 开源协议，允许自由使用、修改、商用，需保留原作者信息（详细协议见LICENSE文件）。

---
© 2026 多无人机协同路径规划项目
