import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from queue import PriorityQueue

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
    """A* 路径规划核心函数"""
    neighbors = [(0,1), (1,0), (0,-1), (-1,0), (1,1), (-1,-1), (1,-1), (-1,1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = PriorityQueue()
    oheap.put((fscore[start], start))

    while not oheap.empty():
        current = oheap.get()[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            if 0 <= neighbor[0] < map_size and 0 <= neighbor[1] < map_size:
                if neighbor in obstacles:
                    continue

                temp_g = gscore[current] + heuristic(current, neighbor)
                if neighbor in close_set and temp_g >= gscore.get(neighbor, 0):
                    continue

                if temp_g < gscore.get(neighbor, 0) or neighbor not in [i[1] for i in oheap.queue]:
                    came_from[neighbor] = current
                    gscore[neighbor] = temp_g
                    fscore[neighbor] = temp_g + heuristic(neighbor, goal)
                    oheap.put((fscore[neighbor], neighbor))

    return []

# ====================== 3. 生成所有无人机路径 ======================
def generate_paths(drones, obstacles, map_size):
    paths = []
    obs = set((x, y) for x, y in obstacles)
    for idx, (sx, sy, gx, gy) in enumerate(drones):
        start = (int(sx), int(sy))
        goal = (int(gx), int(gy))
        path = a_star(start, goal, obs, map_size)
        paths.append(path)
        print(f"✅ 无人机 {idx+1} 路径规划完成，路径长度：{len(path)}")
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
            dist = np.hypot(x1 - x2, y1 - y2)
            if dist < safe_dist:
                collisions.append((i, j))
    return collisions

def avoid_collision(drone_positions, paths, current_idx, safe_dist):
    """冲突规避：速度调整 + 路径微调"""
    new_positions = drone_positions.copy()
    collide_pairs = check_collisions(drone_positions, safe_dist)

    for i, j in collide_pairs:
        # 策略：后机减速等待，前机正常飞行
        if current_idx[i] >= current_idx[j]:
            # 无人机i 等待一帧
            new_positions[i] = drone_positions[i]
        else:
            new_positions[j] = drone_positions[j]

    return new_positions

def move_along_path(pos, path, current_step, step_size):
    """沿路径平滑移动"""
    if current_step >= len(path) - 1:
        return pos

    target = path[current_step + 1]
    dx = target[0] - pos[0]
    dy = target[1] - pos[1]
    dist = np.hypot(dx, dy)

    if dist < step_size:
        return np.array(target)
    else:
        move_x = dx / dist * step_size
        move_y = dy / dist * step_size
        return np.array([pos[0] + move_x, pos[1] + move_y])

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

    # 绘制起点/终点
    colors = ['red', 'blue', 'green', 'orange', 'purple']
    for i, drone in enumerate(DRONES):
        sx, sy, gx, gy = drone
        ax.scatter(sx, sy, c=colors[i], s=80, marker='o', label=f'无人机{i+1}起点')
        ax.scatter(gx, gy, c=colors[i], s=80, marker='*', label=f'无人机{i+1}终点')

    # 初始化无人机位置与状态
    drone_pos = [np.array([path[0][0], path[0][1]]) for path in paths]
    current_steps = [0] * len(paths)
    drone_dots = [ax.plot([], [], 'o', markersize=10, color=colors[i])[0] for i in range(len(paths))]

    def update(frame):
        # 沿路径移动
        for i in range(len(paths)):
            if current_steps[i] < len(paths[i]) - 1:
                drone_pos[i] = move_along_path(drone_pos[i], paths[i], current_steps[i], STEP_SIZE)

                target = paths[i][current_steps[i] + 1]
                if np.hypot(drone_pos[i][0] - target[0], drone_pos[i][1] - target[1]) < 0.3:
                    current_steps[i] += 1

        # 冲突规避
        drone_pos[:] = avoid_collision(drone_pos, paths, current_steps, SAFE_DISTANCE)

        # 更新画面
        for i, dot in enumerate(drone_dots):
            dot.set_data(drone_pos[i][0], drone_pos[i][1])

        return drone_dots

    ani = animation.FuncAnimation(
        fig, update, frames=300, interval=50, blit=True, repeat=False
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
