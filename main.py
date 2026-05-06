import heapq
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei']  # 兼容中文+数学符号
plt.rcParams['axes.unicode_minus'] = False

# ====================== 1. 基础配置 ======================
# 地图大小
MAP_SIZE = 30
# 无人机安全距离（小于这个距离判定为冲突）
SAFE_DISTANCE = 2.0
# 每帧移动步长
STEP_SIZE = 0.8

# 无人机设置：[起点x, 起点y, 终点x, 终点y]
DRONES = [
    [2, 2, 27, 27],   # 无人机1
    [2, 27, 27, 2],   # 无人机2
    [15, 2, 15, 27],  # 无人机3
]

# 障碍物：[x, y]
OBSTACLES = [
    [10, 10], [10, 15], [10, 20],
    [15, 10], [20, 10], [20, 20]
]

# ====================== 2. A* 路径规划算法 ======================
def heuristic(a, b):
    """曼哈顿距离启发函数"""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star(start, goal, obstacles, map_size):
    """A* 路径规划核心函数（基于整数栅格）"""
    neighbors = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]
    obstacles = set(obstacles)
    start = tuple(start)
    goal = tuple(goal)

    open_heap = []
    heapq.heappush(open_heap, (heuristic(start, goal), start))
    open_set = {start}
    came_from = {}
    gscore = {start: 0}

    while open_heap:
        current = heapq.heappop(open_heap)[1]
        open_set.discard(current)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for dx, dy in neighbors:
            neighbor = (current[0] + dx, current[1] + dy)
            # 检查边界
            if not (0 <= neighbor[0] < map_size and 0 <= neighbor[1] < map_size):
                continue
            # 障碍物
            if neighbor in obstacles:
                continue

            tentative_g = gscore[current] + np.hypot(dx, dy)

            if tentative_g < gscore.get(neighbor, float('inf')):
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g
                fscore = tentative_g + heuristic(neighbor, goal)
                if neighbor not in open_set:
                    heapq.heappush(open_heap, (fscore, neighbor))
                    open_set.add(neighbor)
    # 无路径
    return []

def fallback_straight_path(start, goal):
    """当 A* 失败时，使用直线插值生成离散格点路径（作为后备）"""
    sx, sy = start
    gx, gy = goal
    dx = gx - sx
    dy = gy - sy
    steps = max(int(abs(dx)), int(abs(dy)), 1)
    xs = np.linspace(sx, gx, steps + 1)
    ys = np.linspace(sy, gy, steps + 1)
    path = []
    seen = set()
    for x, y in zip(xs, ys):
        ix, iy = int(round(x)), int(round(y))
        if (ix, iy) not in seen:
            seen.add((ix, iy))
            path.append((ix, iy))
    return path

# ====================== 3. 生成所有无人机路径 ======================
def generate_paths(drones, obstacles, map_size):
    paths = []
    obs = set((int(x), int(y)) for x, y in obstacles)
    for idx, (sx, sy, gx, gy) in enumerate(drones):
        start = (int(sx), int(sy))
        goal = (int(gx), int(gy))
        path = a_star(start, goal, obs, map_size)
        if not path:
            # 后备路径，避免空路径导致后续错误
            path = fallback_straight_path(start, goal)
            print(f"⚠️ 无人机 {idx+1} A* 未找到路径，使用直线后备路径，长度：{len(path)}")
        else:
            print(f"✅ 无人机 {idx+1} 路径规划完成，路径长度：{len(path)}")
        paths.append(path)
    return paths

# ====================== 4. 冲突检测与规避核心 ======================
def check_collisions(drone_positions, safe_dist):
    """检测所有无人机之间是否冲突"""
    collisions = []
    n = len(drone_positions)
    for i in range(n):
        x1, y1 = drone_positions[i]
        for j in range(i + 1, n):
            x2, y2 = drone_positions[j]
            dist = np.hypot(float(x1) - float(x2), float(y1) - float(x2) * 0 + float(y1) - float(y2))  # 保持兼容性
            dist = np.hypot(float(x1) - float(x2), float(y1) - float(y2))
            if dist < safe_dist:
                collisions.append((i, j))
    return collisions

def avoid_collision(drone_positions, paths, current_idx, safe_dist):
    """冲突规避：后机减速等待（简单策略）"""
    # 深拷贝位置数组，避免修改原引用
    new_positions = [pos.copy() for pos in drone_positions]
    collide_pairs = check_collisions(drone_positions, safe_dist)

    for i, j in collide_pairs:
        # 规则：到达路径点更多（或步数更大）的被认为是 "后机"，等待一帧
        if current_idx[i] >= current_idx[j]:
            # i 等待，不移动（保持当前位置）
            new_positions[i] = drone_positions[i].copy()
        else:
            new_positions[j] = drone_positions[j].copy()

    return new_positions

def move_along_path(pos, path, current_step, step_size):
    """沿路径平滑移动，pos: numpy array([x,y])"""
    if len(path) == 0:
        return pos
    if current_step >= len(path) - 1:
        # 已到最后节点，保持当前位置（或置为终点）
        return pos

    target = path[current_step + 1]
    tx, ty = float(target[0]), float(target[1])
    dx = tx - float(pos[0])
    dy = ty - float(pos[1])
    dist = np.hypot(dx, dy)

    if dist <= 1e-6:
        return np.array([tx, ty], dtype=float)

    if dist <= step_size:
        # 直接到达下一个离散点
        return np.array([tx, ty], dtype=float)
    else:
        move_x = dx / dist * step_size
        move_y = dy / dist * step_size
        return np.array([pos[0] + move_x, pos[1] + move_y], dtype=float)

# ====================== 5. 动画可视化 ======================
def run_simulation(paths, obstacles):
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(0, MAP_SIZE)
    ax.set_ylim(0, MAP_SIZE)
    ax.set_title("多无人机协同路径规划 + 动态冲突规避", fontsize=14)
    ax.grid(True)

    # 绘制障碍物
    obs_x = [x for x, y in obstacles]
    obs_y = [y for x, y in obstacles]
    ax.scatter(obs_x, obs_y, c='black', s=100, marker='s', label='障碍物')

    # 绘制起点/终点 (只标注一次以避免重复)
    colors = ['red', 'blue', 'green', 'orange', 'purple']
    for i, drone in enumerate(DRONES):
        sx, sy, gx, gy = drone
        ax.scatter(sx, sy, c=colors[i], s=80, marker='o')
        ax.scatter(gx, gy, c=colors[i], s=80, marker='*')
    # 单独添加图例项
    ax.scatter([], [], c='red', s=80, marker='o', label='起点')
    ax.scatter([], [], c='red', s=80, marker='*', label='终点')

    # 初始化无人机位置与状态
    drone_pos = []
    for i, path in enumerate(paths):
        if len(path) >= 1:
            p0 = np.array([float(path[0][0]), float(path[0][1])], dtype=float)
        else:
            # 退回到 DRONES 起点
            sx, sy = DRONES[i][0], DRONES[i][1]
            p0 = np.array([float(sx), float(sy)], dtype=float)
        drone_pos.append(p0)

    current_steps = [0] * len(paths)
    drone_dots = []
    for i in range(len(paths)):
        dot, = ax.plot([], [], 'o', markersize=10, color=colors[i])
        drone_dots.append(dot)

    # 绘制每条路径轨迹（可选）
    for i, path in enumerate(paths):
        px = [pt[0] for pt in path]
        py = [pt[1] for pt in path]
        ax.plot(px, py, linestyle='--', color=colors[i], alpha=0.6)

    def update(frame):
        nonlocal drone_pos, current_steps
        # 先移动
        for i in range(len(paths)):
            if current_steps[i] < len(paths[i]) - 1:
                newp = move_along_path(drone_pos[i], paths[i], current_steps[i], STEP_SIZE)
                drone_pos[i] = newp
                # 如果靠近当前目标格点则推进 step
                target = paths[i][current_steps[i] + 1]
                if np.hypot(drone_pos[i][0] - target[0], drone_pos[i][1] - target[1]) < 0.3:
                    # 抵达下一个离散节点，步数+1
                    current_steps[i] = min(current_steps[i] + 1, len(paths[i]) - 1)

        # 冲突规避（位置修正/等待）
        drone_pos = avoid_collision(drone_pos, paths, current_steps, SAFE_DISTANCE)

        # 更新画面（注意传入序列而非标量）
        for i, dot in enumerate(drone_dots):
            x, y = float(drone_pos[i][0]), float(drone_pos[i][1])
            dot.set_data([x], [y])

        return drone_dots

    # 注：为避免与某些后端的 blitting/resize 冲突，这里使用 blit=False
    ani = animation.FuncAnimation(
        fig, update, frames=600, interval=50, blit=False, repeat=False
    )

    plt.legend(loc='upper right', fontsize=9)
    plt.tight_layout()
    plt.show()

# ====================== 主程序入口 ======================
if __name__ == "__main__":
    print("===== 多无人机协同路径规划与冲突规避系统 =====")
    # 1. 生成路径
    paths = generate_paths(DRONES, OBSTACLES, MAP_SIZE)
    # 2. 启动仿真
    run_simulation(paths, OBSTACLES)
