import json
import math


def load_map(json_path):
    with open(json_path, "r") as f:
        data = json.load(f)

    segments = data["segments"]
    start_point = tuple(data["start_point"])

    # 1. 找出地图边界
    max_x = max(max(seg["start"][0], seg["end"][0]) for seg in segments)
    max_y = max(max(seg["start"][1], seg["end"][1]) for seg in segments)

    width, height = max_x + 1, max_y + 1

    # 2. 初始化二维数组（0=通路）
    grid = [[0 for _ in range(width)] for _ in range(height)]

    # 3. 遍历线段并画到二维数组上（1=障碍）
    for seg in segments:
        x1, y1 = seg["start"]
        x2, y2 = seg["end"]

        if x1 == x2:  # 垂直线
            y_start, y_end = sorted([y1, y2])
            for y in range(y_start, y_end + 1):
                grid[y][x1] = 1
        elif y1 == y2:  # 水平线
            x_start, x_end = sorted([x1, x2])
            for x in range(x_start, x_end + 1):
                grid[y1][x] = 1
        else:
            # 如果未来有斜线，可以加 Bresenham 算法
            raise ValueError(f"非水平/垂直线段: {seg}")

    return grid, start_point


def scale_grid(grid, start_point, scale):
    """
    将原始迷宫按比例放大，同时保持正方形结构。
    :param grid: 原始二维列表，0=通路，1=障碍
    :param start_point: 起点 (x, y)
    :param scale: 放大倍数 (int)
    :return: (new_grid, new_start_point)
    """
    if scale <= 1:
        return grid, start_point

    H = len(grid)
    W = len(grid[0]) if H > 0 else 0
    new_H, new_W = H * scale, W * scale

    new_grid = [[0 for _ in range(new_W)] for _ in range(new_H)]

    for y in range(H):
        for x in range(W):
            val = grid[y][x]
            for dy in range(scale):
                for dx in range(scale):
                    new_grid[y * scale + dy][x * scale + dx] = val

    sx, sy = start_point
    new_start = (sx * scale, sy * scale)

    return new_grid, new_start


def scale_grid_widen_passages(grid, start_point, target_size, wall_thickness=1, free_val=0, wall_val=1):
    """
    将原始迷宫缩放到指定的目标尺寸，同时可以控制墙体厚度。
    - 通路会被适当缩放
    - 墙体按指定厚度绘制，保持连接性

    :param grid: 原始二维列表，free_val=通路，wall_val=障碍
    :param start_point: 起点 (x, y)
    :param target_size: 目标地图尺寸，可以是int（正方形）或tuple(width, height)
    :param wall_thickness: 墙体厚度（像素），必须为正整数
    :param free_val: 通路值（默认 0）
    :param wall_val: 障碍值（默认 1）
    :return: (new_grid, new_start_point)
    """
    if not grid or not grid[0]:
        return grid, start_point

    # 解析目标尺寸
    if isinstance(target_size, int):
        new_W = new_H = target_size
    elif isinstance(target_size, (tuple, list)) and len(target_size) == 2:
        new_W, new_H = int(target_size[0]), int(target_size[1])
    else:
        raise ValueError("target_size必须是int或包含两个元素的tuple/list")

    # 验证参数
    if new_W <= 0 or new_H <= 0:
        raise ValueError("目标尺寸必须为正数")
    if wall_thickness <= 0:
        raise ValueError("墙体厚度必须为正整数")

    H = len(grid)
    W = len(grid[0])

    # 计算缩放比例
    scale_x = new_W / W
    scale_y = new_H / H

    # 初始化为全通路的目标地图
    new_grid = [[free_val for _ in range(new_W)] for _ in range(new_H)]

    def draw_wall_region(center_x, center_y, thickness):
        """在指定中心点周围绘制指定厚度的墙体"""
        half_thickness = thickness // 2
        for dy in range(-half_thickness, thickness - half_thickness):
            for dx in range(-half_thickness, thickness - half_thickness):
                nx, ny = center_x + dx, center_y + dy
                if 0 <= nx < new_W and 0 <= ny < new_H:
                    new_grid[ny][nx] = wall_val

    def draw_wall_line(x1, y1, x2, y2, thickness):
        """绘制指定厚度的墙体线段"""
        # 使用Bresenham算法的变体，为线段的每个点都绘制厚度
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)

        if dx == 0 and dy == 0:
            draw_wall_region(x1, y1, thickness)
            return

        x, y = x1, y1
        x_inc = 1 if x2 > x1 else -1
        y_inc = 1 if y2 > y1 else -1

        if dx > dy:
            error = dx / 2
            while x != x2:
                draw_wall_region(x, y, thickness)
                error -= dy
                if error < 0:
                    y += y_inc
                    error += dx
                x += x_inc
            draw_wall_region(x2, y2, thickness)  # 绘制终点
        else:
            error = dy / 2
            while y != y2:
                draw_wall_region(x, y, thickness)
                error -= dx
                if error < 0:
                    x += x_inc
                    error += dy
                y += y_inc
            draw_wall_region(x2, y2, thickness)  # 绘制终点

    # 扫描原图，处理墙体
    for y in range(H):
        for x in range(W):
            if grid[y][x] == wall_val:
                # 计算在新地图中的对应位置
                new_x = int(round(x * scale_x))
                new_y = int(round(y * scale_y))

                # 确保坐标在有效范围内
                new_x = max(0, min(new_W - 1, new_x))
                new_y = max(0, min(new_H - 1, new_y))

                # 检查原图中的连接关系来决定如何绘制墙体
                left = (x - 1 >= 0 and grid[y][x - 1] == wall_val)
                right = (x + 1 < W and grid[y][x + 1] == wall_val)
                up = (y - 1 >= 0 and grid[y - 1][x] == wall_val)
                down = (y + 1 < H and grid[y + 1][x] == wall_val)

                # 绘制墙体块
                draw_wall_region(new_x, new_y, wall_thickness)

                # 如果有连接关系，绘制连接线以确保连通性
                if right:
                    next_new_x = int(round((x + 1) * scale_x))
                    next_new_x = max(0, min(new_W - 1, next_new_x))
                    if next_new_x > new_x:
                        draw_wall_line(new_x, new_y, next_new_x, new_y, wall_thickness)

                if down:
                    next_new_y = int(round((y + 1) * scale_y))
                    next_new_y = max(0, min(new_H - 1, next_new_y))
                    if next_new_y > new_y:
                        draw_wall_line(new_x, new_y, new_x, next_new_y, wall_thickness)

    # 计算新起点坐标
    sx, sy = start_point
    new_start_x = int(round(sx * scale_x))
    new_start_y = int(round(sy * scale_y))

    # 确保起点在有效范围内
    new_start_x = max(0, min(new_W - 1, new_start_x))
    new_start_y = max(0, min(new_H - 1, new_start_y))

    # 确保起点是通路
    if new_grid[new_start_y][new_start_x] == wall_val:
        # 在起点周围寻找最近的通路
        found = False
        for radius in range(1, max(wall_thickness * 2, 10)):
            if found:
                break
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    candidate_x = new_start_x + dx
                    candidate_y = new_start_y + dy
                    if (0 <= candidate_x < new_W and 0 <= candidate_y < new_H and
                            new_grid[candidate_y][candidate_x] == free_val):
                        new_start_x, new_start_y = candidate_x, candidate_y
                        found = True
                        break

        # 如果仍然找不到通路，强制设置起点为通路
        if not found:
            new_grid[new_start_y][new_start_x] = free_val

    new_start_point = (new_start_x, new_start_y)

    return new_grid, new_start_point


# 测试代码
if __name__ == "__main__":
    # 创建一个简单的测试迷宫
    test_grid = [
        [1, 1, 1, 1, 1],
        [1, 0, 0, 0, 1],
        [1, 0, 1, 0, 1],
        [1, 0, 0, 0, 1],
        [1, 1, 1, 1, 1]
    ]
    test_start = (1, 1)

    # 测试新函数
    print("原始地图尺寸:", len(test_grid[0]), "x", len(test_grid))
    print("原始起点:", test_start)

    # 缩放到800x800，墙厚3像素
    new_grid, new_start = scale_grid_widen_passages(
        test_grid, test_start,
        target_size=20,  # 或者用(800, 600)指定不同的宽高
        wall_thickness=3
    )

    print("新地图尺寸:", len(new_grid[0]), "x", len(new_grid))
    print("新起点:", new_start)

    # 打印一小部分结果
    print("新地图预览:")
    for i, row in enumerate(new_grid):
        if i < 10:  # 只显示前10行
            print("".join(str(c) for c in row[:10]))  # 只显示前10列