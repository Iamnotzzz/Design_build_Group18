# planning.py
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Tuple, Optional, Iterable, Dict, Any,Set
import math
import time
import numpy as np
from collections import deque


FREE_thresh = 254
OBSTACLE_val = 40

# === 你工程里已有的模块（保持这些导入路径按你当前项目结构改名/相对导入）===
# ListGridMap: 维护SLAM占据栅格（0障碍, 255空白, 128未知），提供 m<->pix, SetCarPose, GetCarPose, draw()
# simRover:    读取“累计编码器计数+时间戳”并输出 (dxy_mm, dtheta_deg, dt_s)
# 注意：这里不直接依赖 simRover 的实例；Planner 只负责“生成下一条累计计数+时长”，
#       由你上层把这条命令发给固件/仿真，并在回传里更新累计计数与姿态。
try:
    from sim.ListMap.grid_map import ListGridMap
except Exception:
    # 允许在纯接口场景先不导入
    ListGridMap = Any  # type: ignore

# ======================================================================================
#                                   基础数据结构
# ======================================================================================

class MissionPhase(Enum):
    """高层任务阶段"""
    EXPLORE = auto()       # 自主探索（基于frontier等）
    RETURN_HOME = auto()   # 从任意处回到起点（全局最短路）
    GO_TO_EXIT = auto()    # 从起点沿最短路到终点（比赛需要的最终阶段）
    DONE = auto()          # 任务完成

@dataclass
class Pose:
    """世界系（米/度）位姿，theta度，逆时针为正。"""
    x_m: float
    y_m: float
    theta_deg: float

@dataclass
class WheelGeom:
    """用于里程与编码器换算的底盘标定参数"""
    wheel_radius_mm: float            # 轮半径
    half_axle_len_mm: float           # 半轮距（半轴长）
    ticks_per_cycle: int              # 每圈编码器脉冲数（例如 Rover.ticks_per_cycle=2000）

@dataclass
class CmdTolerance:
    """到达目标点/角的误差容许"""
    pos_tol_m: float = 0.1          # 位置容差
    yaw_tol_deg: float = 20          # 航向角容差

@dataclass
class CTolerance:
    """到达目标点/角的误差容许"""
    pos_tol_m: float = 0.07          # 位置容差
    yaw_tol_deg: float = 40          # 航向角容差


@dataclass
class VelocityLimits:
    """速度/时间限制，用于把目标位姿离散成若干段命令"""
    max_lin_mps: float = 0.25
    max_ang_dps: float = 90.0
    min_segment_time_s: float = 0.2
    max_segment_time_s: float = 1.0

@dataclass
class EncoderTarget:
    """
    一条可下发的控制指令：指定“目标时刻的左右轮**累计**计数”+“应在本段内耗时多久”。
    上层应将这条指令发给底层（或仿真），在该时长内驱动左右轮“累计值”达到此计数。
    """
    cum_left_ticks: int
    cum_right_ticks: int
    duration_ms: int
    target_waypoint: Pose              # 期望抵达的“局部目标”位姿（用于调试/显示）
    tolerance: CmdTolerance

@dataclass
class PlannerStateReport:
    """供上层可视化/调试的状态报告"""
    phase: MissionPhase
    curr_pose: Pose
    next_waypoint: Optional[Pose]
    frontier_count: int
    known_maze_rect: Optional[Tuple[int, int, int, int]]
    has_found_exit: bool
    path_len_nodes: int
    note: str = ""
    # 新增字段
    frontier_points: List[Tuple[int, int]] = None  # 所有前沿点的像素坐标列表
    best_frontier_point: Optional[Tuple[int, int]] = None  # 当前选择的最优前沿点像素坐标
    Spatially_sampled : List[int, int] = None
    exit_pose : Tuple[int,int] = None

# ======================================================================================
#                                   关键子模块接口
# ======================================================================================
class MapRectifier:
    """
    估计“迷宫有效矩形”ROI：
    - 输入：SLAM 栅格 grid[y][x]，0=障碍，255=自由，128=未知
    - 输出：像素系矩形 (x0, y0, x1, y1)，闭区间
    策略：以 “已知=障碍或自由” 的点集做包围盒，并外扩 margin。
         为稳健，ROI 默认“只增不减”（避免因观测瞬时缺失而收缩抖动）。
    """

    def __init__(self,
                 margin_pix: int = 20,          # ROI在四周外扩的像素边距（≥机器人半宽/像素）
                 min_width_pix: int = 5,      # 过滤掉过窄的盒子
                 min_height_pix: int = 5,
                 min_area_pix: int = 400,      # 最小面积过滤
                 allow_shrink: bool = False,   # 是否允许ROI收缩（默认不允许：更稳）
                 max_shrink_per_update: int = 4  # 若允许收缩，每次边界最多内缩像素
                 ):
        self.rect_pix: Optional[Tuple[int,int,int,int]] = None  # (x0,y0,x1,y1)
        self.margin_pix = int(margin_pix)
        self.min_w = int(min_width_pix)
        self.min_h = int(min_height_pix)
        self.min_area = int(min_area_pix)
        self.allow_shrink = bool(allow_shrink)
        self.max_shrink = int(max_shrink_per_update)

    def update_from_grid(self, grid: List[List[int]],ensure_contains: Optional[Tuple[int,int]] = None) -> Optional[Tuple[int,int,int,int]]:
        """
        扫描当前网格，计算“已知像素(0或255)”的包围盒 → 外扩 margin → 与历史ROI融合 → 返回矩形。
        若已知像素太少，则维持上次估计（或返回None）。
        """
        if not grid or not grid[0]:
            return self.rect_pix

        H = len(grid)
        W = len(grid[0])

        # 1) 收集已知（非128）像素的极值
        x0, y0 = W, H
        x1, y1 = -1, -1
        known_count = 0

        # 扫描（纯python双层循环，足够快；若有numpy可替换成向量化）
        for y in range(H):
            row = grid[y]
            for x in range(W):
                v = row[x]
                if v <= 20:  # 考虑到迷宫外围是墙壁，仅使用障碍来外扩roi区域
                    known_count += 1
                    if x < x0: x0 = x
                    if x > x1: x1 = x
                    if y < y0: y0 = y
                    if y > y1: y1 = y

        if known_count == 0:
            # 什么都还没观测到：维持旧值或None
            return self.rect_pix

        # 初始包围盒
        if x1 < x0 or y1 < y0:
            return self.rect_pix  # 理论不会发生（除非全未知），出于安全

        # 2) 最小尺寸/面积过滤：避免刚启动时只有几像素导致ROI太小
        w = x1 - x0 + 1
        h = y1 - y0 + 1
        if w < self.min_w or h < self.min_h or (w * h) < self.min_area:
            # 尺寸太小：可能观测刚开始；不要刷新ROI（防抖）
            return self.rect_pix

        # 3) 外扩 margin 并裁剪到地图范围
        x0m = max(0, x0 +4)
        y0m = max(0, y0  +4)
        x1m = min(W - 1, x1-8 )
        y1m = min(H - 1, y1 -8)
        """
        # 3)b不做外扩
        x0m = max(0, x0)
        y0m = max(0, y0)  # 永远保证迷宫位于起点的上方，所以y0不需要外扩
        x1m = min(W - 1, x1)
        y1m = min(H - 1, y1 )
        """

        if ensure_contains is not None:
            cx, cy = ensure_contains
            x0m = min(x0m, max(0, cx))
            y0m = min(y0m, max(0, cy))
            x1m = max(x1m, min(W - 1, cx))
            y1m = max(y1m, min(H - 1, cy))

        candidate = (x0m, y0m, x1m, y1m)

        # 4) 与历史ROI融合 —— 缺省“只增不减”，可选允许缓慢收缩
        if self.rect_pix is None:
            self.rect_pix = candidate
            return self.rect_pix

        cx0, cy0, cx1, cy1 = candidate
        px0, py0, px1, py1 = self.rect_pix

        # 扩张：取并集（更稳）
        nx0 = min(px0, cx0)
        ny0 = min(py0, cy0)
        nx1 = max(px1, cx1)
        ny1 = max(py1, cy1)

        if self.allow_shrink:
            # 允许缓慢收缩：每次最多内缩 max_shrink 像素，防止抖动
            nx0 = self._soft_shrink(px0, cx0, inward=True)
            ny0 = self._soft_shrink(py0, cy0, inward=True)
            nx1 = self._soft_shrink(px1, cx1, inward=False)
            ny1 = self._soft_shrink(py1, cy1, inward=False)

            # 同时确保不会把盒子变得过小
            if (nx1 - nx0 + 1) < self.min_w or (ny1 - ny0 + 1) < self.min_h:
                nx0, ny0, nx1, ny1 = px0, py0, px1, py1  # 放弃收缩

        self.rect_pix = (nx0, ny0, nx1, ny1)
        return self.rect_pix

    # ---------- 帮助函数 ----------

    def _soft_shrink(self, prev_edge: int, cand_edge: int, inward: bool) -> int:
        """
        prev_edge: 旧ROI的某一条边
        cand_edge: 新候选盒子的对应边
        inward=True 表示左/上边（向内收缩是增大数值），False 表示右/下边（向内收缩是减小数值）
        策略：优先取扩张（更保守）；若候选边比旧边更“向内”，则最多移动 max_shrink 像素。
        """
        if inward:
            # 左/上边：cand_edge > prev_edge 表示向内收缩
            if cand_edge <= prev_edge:
                return cand_edge  # 扩张或等同：直接采用（向外扩更稳）
            else:
                # 只能小幅度向内收
                return min(prev_edge + self.max_shrink, cand_edge)
        else:
            # 右/下边：cand_edge < prev_edge 表示向内收缩
            if cand_edge >= prev_edge:
                return cand_edge  # 扩张或等同：直接采用
            else:
                return max(prev_edge - self.max_shrink, cand_edge)

    def is_inside(self, x_pix: float, y_pix: float) -> bool:
        """像素坐标是否落在‘有效矩形’里（若尚未估计出矩形，则默认True）。"""
        if not self.rect_pix:
            return True
        x0, y0, x1, y1 = self.rect_pix
        return (x0 <= x_pix <= x1) and (y0 <= y_pix <= y1)

    def get_roi(self) -> Optional[Tuple[int, int, int, int]]:
        """返回当前ROI（像素系）。"""
        return self.rect_pix


class FrontierExplorer:
    """
    前沿（frontier）提取与选择器。
    约定：
      - 栅格取值：0=障碍，255=自由，128=未知
      - 仅在 maze_rect 内搜索，避免 ROI 外“无穷未知海”导致的伪前沿
      - 过滤掉过靠近障碍的候选（安全距离像素级）
    选目标：
      - 不用欧氏距离；改用 4-邻接 BFS 的“栅格最短步数”作主代价
      - 加一点信息增益（候选周围未知数量）作为负奖励
      - 最终选择总代价最小的前沿点
    """

    def __init__(self,
                 free_thresh: int = FREE_thresh,        # >= 此灰度视作自由
                 unknown_val: int = 128,        # 未知
                 obstacle_val: int = OBSTACLE_val,         # 障碍
                 obs_clearance_pix: int = 2,    # 与墙体保持的安全像素距离
                 gain_radius_pix: int = 2,      # 计算信息增益的邻域半径
                 gain_weight: float = 0.25      # 信息增益权重（越大越偏好“更可能带来新信息”的前沿）
                 ):
        self.free_th = int(free_thresh)
        self.unknown_v = int(unknown_val)
        self.obst_v = int(obstacle_val)
        self.clear_r = int(obs_clearance_pix)
        self.gain_r = int(gain_radius_pix)
        self.gain_w = float(gain_weight)

    # -------------------- 前沿检测 --------------------

    def detect_frontiers(self, grid: List[List[int]],
                         maze_rect: Optional[Tuple[int,int,int,int]]) -> List[Tuple[int,int]]:
        """
        返回前沿像素列表（x_pix,y_pix）。
        规则：
          1) 仅在 maze_rect 内搜索（若为 None 则全图）
          2) 某点为“自由”(>=free_thresh) 且 8邻域存在“未知”(==unknown_val)
          3) 该点与障碍(==0)需满足像素级安全距离 obs_clearance_pix
        """
        H = len(grid)
        W = len(grid[0]) if H else 0
        if H == 0 or W == 0:
            return []

        if maze_rect is None:
            x0, y0, x1, y1 = 0, 0, W-1, H-1
        else:
            x0, y0, x1, y1 = maze_rect
            x0 = max(0, x0); y0 = max(0, y0); x1 = min(W-1, x1); y1 = min(H-1, y1)

        def has_unknown_ngb(x:int,y:int) -> bool:
            # 8 邻域只要有一个未知即算“前沿”
            for jy in (-1,0,1):
                for ix in (-1,0,1):
                    if ix==0 and jy==0: continue
                    xn, yn = x+ix, y+jy
                    if 0 <= xn < W and 0 <= yn < H and self.obst_v<grid[yn][xn] < self.free_th:
                        return True
            return False

        def safe_from_obstacle(x:int,y:int) -> bool:
            # 圆形邻域内不得有障碍
            r = self.clear_r
            rr = r*r
            for jy in range(-r, r+1):
                for ix in range(-r, r+1):
                    if ix*ix + jy*jy > rr:
                        continue
                    xn, yn = x+ix, y+jy
                    if 0 <= xn < W and 0 <= yn < H and grid[yn][xn] < self.obst_v:
                        return False
            return True

        res: List[Tuple[int,int]] = []
        for y in range(max(y0,1), min(y1, H-2)+1):       # 避免访问边界外邻域
            row = grid[y]
            for x in range(max(x0,1), min(x1, W-2)+1):
                v = row[x]
                if v >= self.free_th and has_unknown_ngb(x,y) and safe_from_obstacle(x,y):
                    res.append((x,y))
        return res


    def pick_goal(self,
                  curr_pix: Tuple[float, float],
                  candidates: List[Tuple[int, int]],
                  exclude_radius_pix: float = 3
                  ) -> Optional[Tuple[int, int]]:
        """
        使用欧氏距离选择最近前沿点。
        - curr_pix:  当前像素坐标 (cx, cy)
        - candidates: detect_frontiers() 的输出
        返回：最佳前沿点 (x, y) 或 None
        注：为避免并列时频繁抖动，加入次要排序键（如坐标字典序），保证稳定性。
        """
        if not candidates:
            return None

        cx, cy = curr_pix
        r2 = exclude_radius_pix * exclude_radius_pix

        # 1) 过滤掉与当前像素“太近”的候选（包含同一像素格）
        filtered = [p for p in candidates if ((p[0] - cx) ** 2 + (p[1] - cy) ** 2) > r2]

        # 主键：欧氏距离平方；次键：y，再次键：x（稳定选择，防止并列抖动）
        best = min(
            filtered,
            key=lambda p: ((p[0] - cx) ** 2 + (p[1] - cy) ** 2, p[1], p[0])
        )
        return best


class OptimizedFrontierExplorer:
    """
    优化的前沿探索器，解决选择最优前沿点的性能和策略问题
    """

    def __init__(self,
                 free_thresh: int = FREE_thresh,
                 unknown_val: int = 128,
                 obstacle_val: int = OBSTACLE_val,
                 obs_clearance_pix: int = 10,
                 gain_radius_pix: int = 8,  # 信息增益计算半径
                 connectivity_check_radius: int = 200,  # 连通性检查半径
                 min_distance_pix: int = 30,  # 最小选择距离，避免打转
                 max_distance_pix: int = 200,  # 最大考虑距离，避免不连通
                 spatial_sample_grid: int = 10,  # 空间采样网格大小
                 max_candidates_to_eval: int = 50,
                 min_wall_distance_pix: int = 20,
                 goal_bias_weight: float = 0.6):  # 新增参数：最小墙壁距离):  # 最多评估的候选点数

        self.free_th = int(free_thresh)
        self.unknown_v = int(unknown_val)
        self.obst_v = int(obstacle_val)
        self.clear_r = int(obs_clearance_pix)
        self.gain_r = int(gain_radius_pix)
        self.conn_r = int(connectivity_check_radius)
        self.min_dist = int(min_distance_pix)
        self.max_dist = int(max_distance_pix)
        self.sample_grid = int(spatial_sample_grid)
        self.max_eval = int(max_candidates_to_eval)
        self.SPATIALLY = None
        self.min_wall_dist = int(min_wall_distance_pix)  # 新增：最小墙壁距离
        self.goal_bias_weight = goal_bias_weight  # 新增

    def pick_goal(self,
                  curr_pix: Tuple[float, float],
                  candidates: List[Tuple[int, int]],
                  grid: List[List[int]],
                  goal_pix: Optional[Tuple[int, int]] = None,
                  exclude_radius_pix: float = 3,
                  maze_rect: Optional[Tuple[int, int, int, int]] = None) -> List[Tuple[int, int]]:
        """
        优化的前沿点选择：连通性 + 信息增益 + 性能优化
        """
        if not candidates:
            return []

        start_time = time.time()
        cx, cy = int(curr_pix[0]), int(curr_pix[1])

        # 第一层筛选：距离过滤
        distance_filtered = self._filter_by_distance(candidates, (cx, cy), exclude_radius_pix)
        if not distance_filtered:
            return []

        # 第二层筛选：空间采样减少候选点
        spatially_sampled = self._spatial_sampling(distance_filtered)

        #self.SPATIALLY = spatially_sampled
        # 第三层筛选：连通性检查
        connected_candidates = self._filter_by_connectivity_simple(spatially_sampled, (cx, cy), grid,maze_rect)
        if not connected_candidates:
            # 如果没有连通的候选点，退化到最近距离选择
            return []
        # 第3.5层 检查候选前沿点是否距离墙壁过近

        wall_safe_candidates = self._filter_by_wall_distance(
            connected_candidates,
            grid,
            self.min_wall_dist,
            maze_rect
        )


        # 【新增】第四层筛选：如果提供了终点，则按朝向终点优先级排序
        if goal_pix is not None:
            goal_biased_candidates = self._filter_by_goal_proximity(
                wall_safe_candidates, (cx, cy), goal_pix
            )
        else:
            goal_biased_candidates = connected_candidates

        self.SPATIALLY = connected_candidates


        """
        # 第五层：信息增益评估
        best_candidate = self._select_by_information_gain(connected_candidates, grid)
        """

        elapsed = time.time() - start_time
        print(
            f"前沿选择耗时: {elapsed:.3f}s, 候选点: {len(candidates)} -> {len(distance_filtered)} -> {len(spatially_sampled)} -> {len(connected_candidates)}->{len(wall_safe_candidates)}->{len(goal_biased_candidates)}")

        return goal_biased_candidates

    def _filter_by_goal_proximity(self,
                                  candidates: List[Tuple[int, int]],
                                  curr_pix: Tuple[int, int],
                                  goal_pix: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        【新增方法】根据与终点的接近程度筛选前沿点

        策略：
        1. 计算每个候选点到终点的距离
        2. 计算候选点是否在"当前位置→终点"方向的前方（向量点积）
        3. 综合评分：距离越近、方向越对齐，得分越高
        4. 返回得分排序后的候选点列表

        Args:
            candidates: 候选前沿点列表
            curr_pix: 当前位置像素坐标
            goal_pix: 终点像素坐标

        Returns:
            按终点接近度排序的候选点列表
        """
        if not candidates:
            return []

        cx, cy = curr_pix
        gx, gy = goal_pix

        # 当前位置到终点的向量
        goal_vec = (gx - cx, gy - cy)
        goal_dist = math.sqrt(goal_vec[0] ** 2 + goal_vec[1] ** 2)

        if goal_dist < 1e-6:
            # 当前位置已接近终点，直接返回原列表
            return candidates

        # 归一化方向向量
        goal_dir = (goal_vec[0] / goal_dist, goal_vec[1] / goal_dist)

        scored_candidates = []

        for px, py in candidates:
            # 候选点到终点的距离
            dist_to_goal = math.sqrt((px - gx) ** 2 + (py - gy) ** 2)

            # 当前位置→候选点的向量
            cand_vec = (px - cx, py - cy)
            cand_dist = math.sqrt(cand_vec[0] ** 2 + cand_vec[1] ** 2)

            if cand_dist < 1e-6:
                continue

            # 方向对齐度（点积，范围[-1, 1]，1表示完全朝向终点）
            alignment = (cand_vec[0] * goal_dir[0] + cand_vec[1] * goal_dir[1]) / cand_dist

            # 综合得分：
            # - 距离终点越近越好（归一化到[0,1]）
            # - 方向越对齐越好（归一化到[0,1]）
            # - 使用可配置的权重平衡
            distance_score = 1.0 / (1.0 + dist_to_goal / 100.0)  # 距离得分
            alignment_score = (alignment + 1.0) / 2.0  # 对齐得分：[-1,1] → [0,1]

            # 综合得分（权重可调）
            final_score = (self.goal_bias_weight * distance_score +
                           (1 - self.goal_bias_weight) * alignment_score)

            scored_candidates.append(((px, py), final_score))

        # 按得分降序排序
        scored_candidates.sort(key=lambda x: -x[1])

        # 返回排序后的坐标列表
        result = [coord for coord, _ in scored_candidates]

        print(f"终点导向筛选: {len(candidates)}个候选点, "
              f"最佳得分={scored_candidates[0][1]:.3f}")

        return result

    def _filter_by_wall_distance(self,
                                 candidates: List[Tuple[int, int]],
                                 grid: List[List[int]],
                                 min_wall_distance_pix: int,
                                 maze_rect: Optional[Tuple[int, int, int, int]] = None) -> List[Tuple[int, int]]:
        """
        剔除距离墙壁过近的前沿点

        策略：对每个候选点，检查其周围圆形范围内是否存在障碍物（墙壁）
        如果范围内有墙壁，说明该点太靠近墙壁，应该被剔除

        Args:
            candidates: 候选前沿点列表 [(x_pix, y_pix), ...]
            grid: 栅格地图 grid[y][x]
            min_wall_distance_pix: 与墙壁的最小安全距离（像素半径）
            maze_rect: 迷宫有效区域 (x0, y0, x1, y1)，用于边界检查

        Returns:
            筛选后的安全前沿点列表
        """
        if not candidates or min_wall_distance_pix <= 0:
            return candidates

        H = len(grid)
        W = len(grid[0]) if H > 0 else 0

        if W == 0 or H == 0:
            return candidates

        # 获取ROI范围（避免检查超出迷宫的区域）
        if maze_rect is not None:
            x0, y0, x1, y1 = maze_rect
            x0 = max(0, x0)
            y0 = max(0, y0)
            x1 = min(W - 1, x1)
            y1 = min(H - 1, y1)
        else:
            x0, y0, x1, y1 = 0, 0, W - 1, H - 1

        safe_candidates = []
        r = min_wall_distance_pix
        r_squared = r * r

        for px, py in candidates:
            is_safe = True

            # 在圆形范围内检查是否有墙壁
            for dy in range(-r, r + 1):
                if not is_safe:
                    break

                for dx in range(-r, r + 1):
                    # 圆形范围判断：只检查圆内的点
                    if dx * dx + dy * dy > r_squared:
                        continue

                    nx, ny = px + dx, py + dy

                    # 边界检查：确保不越界
                    if not (0 <= nx < W and 0 <= ny < H):
                        continue

                    # ROI检查：只在有效迷宫区域内检查
                    if not (x0 <= nx <= x1 and y0 <= ny <= y1):
                        continue

                    # 墙壁检测：如果该位置是障碍物，标记为不安全
                    if grid[ny][nx] <= self.obst_v:
                        is_safe = False
                        break

            # 如果该点周围没有墙壁，则认为是安全的
            if is_safe:
                safe_candidates.append((px, py))

        return safe_candidates

    def _filter_by_distance(self, candidates: List[Tuple[int, int]],
                            curr_pos: Tuple[int, int],
                            exclude_radius: float) -> List[Tuple[int, int]]:
        """距离筛选：排除太近和太远的点"""
        cx, cy = curr_pos
        exclude_r2 = exclude_radius * exclude_radius
        min_r2 = self.min_dist * self.min_dist
        max_r2 = self.max_dist * self.max_dist

        filtered = []
        for px, py in candidates:
            dist2 = (px - cx) ** 2 + (py - cy) ** 2
            if exclude_r2 < dist2  and dist2 >= min_r2:
                filtered.append((px, py))

        return filtered

    def _spatial_sampling(self, candidates: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """空间采样：在网格中每个格子只保留一个候选点"""
        if len(candidates) <= self.max_eval:
            return candidates

        # 将空间分割为网格，每个网格保留一个点
        grid_dict = {}
        for px, py in candidates:
            grid_x = px // self.sample_grid
            grid_y = py // self.sample_grid
            grid_key = (grid_x, grid_y)

            if grid_key not in grid_dict:
                grid_dict[grid_key] = []
            grid_dict[grid_key].append((px, py))

        # 每个网格选择一个代表点（可以是随机选择或者选择网格中心附近的）
        sampled = []
        for grid_points in grid_dict.values():
            # 选择网格内的第一个点作为代表（也可以用其他策略）
            sampled.append(grid_points[0])


        return sampled

    def _filter_by_connectivity_simple(self, candidates, curr_pos, grid,maze_rect: Optional[Tuple[int, int, int, int]] = None):
        """
        改进版本：基于厚实连通区域的前沿点筛选
        1. 从机器人当前位置开始BFS扩展厚实连通区域
        2. 筛选出落在该区域内的前沿点
        3. 如果结果太少则降级到普通连通性检查
        """
        from collections import deque

        W = len(grid[0])
        H = len(grid)

        x0 = maze_rect[0]
        y0 = maze_rect[1]
        x1 = maze_rect[2]
        y1 = maze_rect[3]

        def is_robust_free(x, y):
            """判断该点是否在'厚实'的自由区域内（参考原is_robust_free逻辑）"""
            free_count = 0
            total = 0
            for dy in range(-5, 6):
                for dx in range(-5, 6):
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < W and 0 <= ny < H and x0<=nx<x1 and y0<=ny<y1:
                        total += 1
                        if grid[ny][nx] >= self.free_th:
                            free_count += 1
            return total > 0 and free_count / total > 0.7 # 稍微降低阈值增加容忍度 2.json的迷宫要改成0.7

        # 1. 找到有效的起始点
        start_x, start_y = int(curr_pos[0]), int(curr_pos[1])

        # 如果当前位置不在厚实区域，在附近搜索
        if not (0 <= start_x < W and 0 <= start_y < H) or not is_robust_free(start_x, start_y):
            print("当前位置不在厚实区域，在附近搜索")
            for r in range(1, 8):
                found = False
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        if dx * dx + dy * dy > r * r:
                            continue
                        nx, ny = start_x + dx, start_y + dy
                        if 0 <= nx < W and 0 <= ny < H and x0<=nx<x1 and y0<=ny<y1 and is_robust_free(nx, ny):
                            start_x, start_y = nx, ny
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            else:
                # 找不到厚实起点，降级处理
                print("无法找到厚实起点，降级处理")
                return self._fallback_connectivity_check(candidates, curr_pos, grid)

        # 2. BFS扩展厚实连通区域
        robust_region = set()
        queue = deque([(start_x, start_y)])
        robust_region.add((start_x, start_y))

        # 使用8连通扩展，提高连通性
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (1, -1), (-1, 1), (1, 1)]

        while queue:
            x, y = queue.popleft()

            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                if (nx, ny) in robust_region:
                    continue

                if not (0 <= nx < W and 0 <= ny < H and x0<=nx<x1 and y0<=ny<y1):
                    continue

                # 基本自由空间检查
                if grid[ny][nx] < self.free_th:
                    continue

                # 厚实性检查
                if not is_robust_free(nx, ny):
                    continue

                robust_region.add((nx, ny))
                queue.append((nx, ny))

        # 3. 筛选在厚实连通区域内的前沿点
        qualified = []
        for px, py in candidates:
            if (px, py) in robust_region:
                qualified.append((px, py))

        # 4. 容错机制：结果太少时降级处理
        min_required = max(1, len(candidates) // 5)  # 至少保留20%或1个
        if len(qualified) < min_required:
            print(f"厚实连通区域前沿点过少({len(qualified)}/{len(candidates)})，")

        if not qualified:
            print("没有connectivity")
        return qualified if qualified else candidates[:2]

    def _fallback_connectivity_check(self, candidates, curr_pos, grid):
        """降级方案：使用普通连通性检查（不要求厚实）"""
        from collections import deque

        W = len(grid[0])
        H = len(grid)

        def is_basic_free(x, y):
            """基本自由空间检查"""
            return 0 <= x < W and 0 <= y < H and grid[y][x] >= self.free_th

        # 寻找有效起始点
        start_x, start_y = int(curr_pos[0]), int(curr_pos[1])

        if not is_basic_free(start_x, start_y):
            # 在附近寻找最近的自由空间
            for r in range(1, 10):
                found = False
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        if dx * dx + dy * dy > r * r:
                            continue
                        nx, ny = start_x + dx, start_y + dy
                        if is_basic_free(nx, ny):
                            start_x, start_y = nx, ny
                            found = True
                            break
                    if found:
                        break
                if found:
                    break
            else:
                # 完全找不到连通起点，返回距离最近的几个前沿点
                return candidates[:2]

        # BFS扩展普通连通区域
        basic_region = set()
        queue = deque([(start_x, start_y)])
        basic_region.add((start_x, start_y))

        # 使用4连通，更保守
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        while queue:
            x, y = queue.popleft()

            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                if (nx, ny) in basic_region:
                    continue

                if not is_basic_free(nx, ny):
                    continue

                basic_region.add((nx, ny))
                queue.append((nx, ny))

        # 筛选在普通连通区域内的前沿点
        qualified = []
        for px, py in candidates:
            if (px, py) in basic_region:
                qualified.append((px, py))

        return qualified if qualified else candidates[:2]

    def _select_by_information_gain(self, candidates: List[Tuple[int, int]],
                                    grid: List[List[int]]) -> List[Tuple[int, int]]:
        """基于信息增益选择最佳候选点"""
        if len(candidates) == 1:
            return candidates

        H, W = len(grid), len(grid[0])
        best_candidate = None
        best_score = -1
        scored_candidates: List[Tuple[Tuple[int, int], float]] = []

        for px, py in candidates:
            # 计算该点周围的信息增益（未知像素密度）
            unknown_count = 0
            total_count = 0

            # 在gain_r半径内采样检查未知像素
            sample_step = max(1, self.gain_r // 4)  # 采样步长，减少计算量

            for dy in range(-self.gain_r, self.gain_r + 1, sample_step):
                for dx in range(-self.gain_r, self.gain_r + 1, sample_step):
                    if dx * dx + dy * dy > self.gain_r * self.gain_r:
                        continue

                    nx, ny = px + dx, py + dy
                    if 0 <= nx < W and 0 <= ny < H:
                        total_count += 1
                        if self.obst_v < grid[ny][nx] < self.free_th:  # 未知区域
                            unknown_count += 1

            if total_count == 0:
                continue

            # 信息增益 = 未知像素比例，同时考虑距离因子（稍微偏向近一点的）
            unknown_ratio = unknown_count / total_count

            """
            # 添加小的距离惩罚，避免选择过远的点
            # 注意：这里curr_pix可能是浮点数，需要转换
            curr_x = int(curr_pix[0]) if hasattr(curr_pix, '__len__') else curr_pix[0]
            curr_y = int(curr_pix[1]) if hasattr(curr_pix, '__len__') else curr_pix[1]
            distance = math.sqrt((px - curr_x) ** 2 + (py - curr_y) ** 2)
            distance_factor = 1.0 / (1.0 + distance / 100.0)  # 距离归一化因子
            """

            score = unknown_ratio
            scored_candidates.append(((px, py), score))
            """
            if score > best_score:
                best_score = score
                best_candidate = (px, py)
            """
        scored_candidates.sort(key=lambda item: (-item[1], item[0][1], item[0][0]))

        #return best_candidate if best_candidate else candidates[0]
        return [coord for coord, _ in scored_candidates]

    def detect_frontiers(self, grid: List[List[int]],
                         maze_rect: Optional[Tuple[int, int, int, int]]) -> List[Tuple[int, int]]:
        """
        前沿检测方法保持不变，但可以在这里添加一些预筛选
        """
        H = len(grid)
        W = len(grid[0]) if H else 0
        if H == 0 or W == 0:
            return []

        if maze_rect is None:
            x0, y0, x1, y1 = 0, 0, W - 1, H - 1
        else:
            x0, y0, x1, y1 = maze_rect
            x0 = max(0, x0);
            y0 = max(0, y0);
            x1 = min(W - 1, x1);
            y1 = min(H - 1, y1)

        def has_unknown_ngb(x: int, y: int) -> bool:
            for jy in (-1, 0, 1):
                for ix in (-1, 0, 1):
                    if ix == 0 and jy == 0: continue
                    xn, yn = x + ix, y + jy
                    if 0 <= xn < W and 0 <= yn < H and self.obst_v  < grid[yn][xn] < self.free_th:
                        return True
            return False

        def safe_from_obstacle(x: int, y: int) -> bool:
            r = self.clear_r
            rr = r * r
            for jy in range(-r, r + 1):
                for ix in range(-r, r + 1):
                    if ix * ix + jy * jy > rr:
                        continue
                    xn, yn = x + ix, y + jy
                    if 0 <= xn < W and 0 <= yn < H and grid[yn][xn] <= self.obst_v:
                        return False
            return True

        res: List[Tuple[int, int]] = []
        for y in range(max(y0, 1), min(y1, H - 2) + 1):
            row = grid[y]
            for x in range(max(x0, 1), min(x1, W - 2) + 1):
                v = row[x]
                if v >= self.free_th and has_unknown_ngb(x, y) and safe_from_obstacle(x, y):
                    res.append((x, y))

        return res


class CoordinateConverter:
    """
    坐标系转换器：世界坐标系 ↔ SLAM地图像素坐标系

    世界坐标系：小车起点为原点(0,0)，单位米，右手坐标系（X右Y前）
    SLAM坐标系：(map_size_pixels/2, map_size_pixels/2)为起点，单位像素
    """

    def __init__(self, map_size_pixels: int, map_size_meters: float):
        """
        Args:
            map_size_pixels: 地图像素尺寸（如800）
            map_size_meters: 地图实际尺寸（如6米）
        """
        self.map_size_pixels = map_size_pixels
        self.map_size_meters = map_size_meters
        self.meters_per_pixel = map_size_meters / map_size_pixels

        # SLAM地图中的起点像素坐标
        self.origin_pix_x = map_size_pixels / 2.0
        self.origin_pix_y = map_size_pixels / 2.0

    def world_to_pixel(self, x_m: float, y_m: float) -> Tuple[int, int]:
        """
        世界坐标（米）→ SLAM像素坐标

        Args:
            x_m, y_m: 世界坐标系下的坐标（米）

        Returns:
            (x_pix, y_pix): 像素坐标
        """
        # 世界坐标以起点为原点，需要加上地图中心偏移
        x_pix = self.origin_pix_x + (x_m / self.meters_per_pixel)
        y_pix = self.origin_pix_y + (y_m / self.meters_per_pixel)
        return (x_pix, y_pix)

    def pixel_to_world(self, x_pix: float, y_pix: float) -> Tuple[float, float]:
        """
        SLAM像素坐标 → 世界坐标（米）

        Args:
            x_pix, y_pix: SLAM地图像素坐标

        Returns:
            (x_m, y_m): 世界坐标系下的坐标（米）
        """
        x_m = (x_pix - self.origin_pix_x) * self.meters_per_pixel
        y_m = (y_pix - self.origin_pix_y) * self.meters_per_pixel
        return (x_m, y_m)

    def is_within_map(self, x_pix: float, y_pix: float) -> bool:
        """检查像素坐标是否在地图范围内"""
        return (0 <= x_pix < self.map_size_pixels and
                0 <= y_pix < self.map_size_pixels)


class GoalReachabilityChecker:
    """
    终点可达性检查器

    功能：
    1. 判断从当前位置到终点是否存在可行路径
    2. 使用简化的连通性检查（BFS）
    3. 考虑地图的已知区域和障碍物
    """

    def __init__(self,
                 free_thresh: int = FREE_thresh,
                 unknown_val: int = 128,
                 obstacle_val: int = OBSTACLE_val,
                 check_radius_pix: int = 5,  # 终点周围需要自由的半径
                 path_check_interval: int = 1):  # 路径检查间隔（帧数）
        """
        Args:
            free_thresh: 自由空间阈值
            unknown_val: 未知区域值
            obstacle_val: 障碍物值
            check_radius_pix: 终点周围的安全半径（像素）
            path_check_interval: 每隔几帧检查一次路径（避免频繁计算）
        """
        self.free_th = int(free_thresh)
        self.unknown_v = int(unknown_val)
        self.obst_v = int(obstacle_val)
        self.check_radius = int(check_radius_pix)
        self.check_interval = int(path_check_interval)

        self._frame_count = 0  # 帧计数器
        self._last_check_result = False  # 上次检查结果（缓存）

    def is_goal_reachable(self,
                          grid: List[List[int]],
                          curr_pix: Tuple[int, int],
                          goal_pix: Tuple[int, int],
                          maze_rect: Optional[Tuple[int, int, int, int]] = None,
                          force_check: bool = True) -> bool:
        """
        检查终点是否可达

        Args:
            grid: 占据栅格地图
            curr_pix: 当前位置（像素）
            goal_pix: 终点位置（像素）
            maze_rect: 有效地图区域
            force_check: 是否强制检查（忽略帧间隔）

        Returns:
            True: 终点可达，应该直接前往终点
            False: 终点不可达，继续探索
        """
        # 帧间隔控制（减少计算开销）
        self._frame_count += 1
        if not force_check and self._frame_count % self.check_interval != 0:
            return self._last_check_result

        H = len(grid)
        W = len(grid[0]) if H > 0 else 0

        if W == 0 or H == 0:
            return False

        gx, gy = int(round(goal_pix[0])), int(round(goal_pix[1]))

        # 检查1：终点是否在地图范围内
        if not (0 <= gx < W and 0 <= gy < H):
            print(f"终点({gx},{gy})超出地图范围")
            self._last_check_result = False
            return False

        # 检查2：终点周围是否为自由空间
        if not self._is_goal_area_free(grid, gx, gy):
            print(f"终点区域未探索或有障碍物")
            self._last_check_result = False
            return False

        # 检查3：从当前位置到终点是否存在路径
        cx, cy = int(round(curr_pix[0])), int(round(curr_pix[1]))

        if self._has_path_bfs(grid, (cx, cy), (gx, gy), maze_rect):
            print(f"✓ 终点可达！当前({cx},{cy}) -> 终点({gx},{gy})")
            self._last_check_result = True
            return True
        else:
            print(f"✗ 终点尚不可达，继续探索")
            self._last_check_result = False
            return False

    def _is_goal_area_free(self, grid: List[List[int]],
                           gx: int, gy: int) -> bool:
        """
        检查终点周围是否为已知的自由空间

        策略：
        - 终点本身必须是自由空间
        - 终点周围一定半径内大部分区域应该是自由空间
        """
        H, W = len(grid), len(grid[0])

        # 检查终点本身
        if not (0 <= gx < W and 0 <= gy < H):
            return False

        if grid[gy][gx] < self.free_th:
            return False  # 终点不是自由空间

        # 检查周围区域
        r = self.check_radius
        free_count = 0
        total_count = 0

        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r * r:
                    continue

                nx, ny = gx + dx, gy + dy
                if 0 <= nx < W and 0 <= ny < H:
                    total_count += 1
                    if grid[ny][nx] >= self.free_th:
                        free_count += 1

        # 至少70%的周围区域是自由空间
        if total_count == 0:
            return False

        free_ratio = free_count / total_count
        return free_ratio >= 0.7

    def _has_path_bfs(self,
                      grid: List[List[int]],
                      start: Tuple[int, int],
                      goal: Tuple[int, int],
                      maze_rect: Optional[Tuple[int, int, int, int]]) -> bool:
        """
        BFS检查路径连通性（轻量级版本，比A*快）

        Args:
            grid: 地图
            start: 起点像素坐标
            goal: 终点像素坐标
            maze_rect: 搜索范围限制

        Returns:
            True: 存在路径, False: 不存在路径
        """
        from collections import deque

        H, W = len(grid), len(grid[0])

        # ROI范围
        if maze_rect:
            x0, y0, x1, y1 = maze_rect
            x0 = max(0, x0)
            y0 = max(0, y0)
            x1 = min(W - 1, x1)
            y1 = min(H - 1, y1)
        else:
            x0, y0, x1, y1 = 0, 0, W - 1, H - 1

        sx, sy = start
        gx, gy = goal

        # 边界检查
        if not (x0 <= sx <= x1 and y0 <= sy <= y1):
            return False
        if not (x0 <= gx <= x1 and y0 <= gy <= y1):
            return False

        # BFS
        queue = deque([(sx, sy)])
        visited = {(sx, sy)}

        # 4连通（简化版，更快）
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1)]

        max_iterations = 5000  # 防止无限循环
        iterations = 0

        while queue and iterations < max_iterations:
            iterations += 1
            x, y = queue.popleft()

            # 到达终点
            if abs(x - gx) <= 8 and abs(y - gy) <= 8:
                return True

            for dx, dy in directions:
                nx, ny = x + dx, y + dy

                if (nx, ny) in visited:
                    continue

                # 范围检查
                if not (x0 <= nx <= x1 and y0 <= ny <= y1):
                    continue

                # 可通行检查：自由空间
                if grid[ny][nx] < self.free_th:
                    continue

                visited.add((nx, ny))
                queue.append((nx, ny))

        return False

class GlobalPathPlanner:
    """
    在占据栅格上用 A* 规划最短路（像素为节点）。
    - 仅在 maze_rect (x0,y0,x1,y1) 内搜索；若为 None 则全图
    - 可选 4/8 邻接；8 邻接时默认禁止“拐角穿越”(no-corner-cut)
    - 仅把“自由”视作可通行：grid[y][x] >= free_thresh
    - 对障碍做像素级膨胀（inflate_radius_pix），提高安全裕度
    - 若起点/终点不在可通行栅格上，会在近邻范围内搜索最近可通行点作为替代
    返回：像素路径（含起点与终点），或空列表表示不可达
    """

    def __init__(self,
                 allow_diagonal: bool = True,
                 inflate_radius_pix: int = 12,
                 free_thresh: int = FREE_thresh,         # >= 此灰度视作自由
                 unknown_val: int = 128,         # 未知
                 obstacle_val: int = OBSTACLE_val,          # 障碍
                 max_snap_radius: int = 18,        # 起/终点就近“吸附”到可通行格的最大半径
                 # 新增简化参数
                 enable_path_simplification: bool = True,
                 target_waypoint_distance_pix: int = 7
                 ):
        self.allow_diag = bool(allow_diagonal)
        self.inflate_r = int(max(0, inflate_radius_pix))
        self.free_th = int(free_thresh)
        self.unknown_v = int(unknown_val)
        self.obst_v = int(obstacle_val)
        self.max_snap_r = int(max(0, max_snap_radius))

        # 新增属性
        self.enable_simplification = enable_path_simplification
        self.target_distance = target_waypoint_distance_pix

    # -------------------- 公有接口 --------------------

    def _plan_path(self,
                  grid: List[List[int]],
                  start_pix: Tuple[int, int],
                  goal_pix: Tuple[int, int],
                  maze_rect: Optional[Tuple[int, int, int, int]]) -> List[Tuple[int, int]]:
        """
        返回像素路径（含 start 与 goal）。若不可达返回 []。
        """
        if not grid or not grid[0]:
            return []

        H = len(grid)
        W = len(grid[0])

        # 1) 约束 ROI
        if maze_rect is None:
            x0, y0, x1, y1 = 0, 0, W - 1, H - 1
        else:
            x0, y0, x1, y1 = maze_rect
            x0 = max(0, min(x0, W - 1))
            x1 = max(0, min(x1, W - 1))
            y0 = max(0, min(y0, H - 1))
            y1 = max(0, min(y1, H - 1))
            if x1 < x0 or y1 < y0:
                return []

        # 2) 在 ROI 内构建“可通行掩模”并对障碍做膨胀
        passable, roi_W, roi_H = self._build_passable_with_inflation(grid, (x0, y0, x1, y1))

        # 3) 将 start/goal 转到 ROI 局部坐标，并吸附到最近可通行点
        sx, sy = start_pix
        gx, gy = goal_pix
        if not (x0 <= sx <= x1 and y0 <= sy <= y1 and x0 <= gx <= x1 and y0 <= gy <= y1):
            # 起终点必须落在 ROI 内；否则不可达
            return []

        lsx, lsy = sx - x0, sy - y0
        lgx, lgy = gx - x0, gy - y0

        if not self._is_inside(lsx, lsy, roi_W, roi_H) or not self._is_inside(lgx, lgy, roi_W, roi_H):
            return []
        #print(f"ori:lsx, lsy{lsx, lsy},lgx, lgy{lgx, lgy}")
        if not passable[lsy][lsx]:
            print("小车当前点不在passable里")
        if not passable[lgy][lgx]:
            print("目标点不在passable里")

        """
        #吸附到最近可通行栅格
        if not passable[lsy][lsx]:
            snap = self._snap_to_nearest_passable(passable, lsx, lsy)
            if snap is None:
                return []
            lsx, lsy = snap

        if not passable[lgy][lgx]:
            snap = self._snap_to_nearest_passable(passable, lgx, lgy)
            if snap is None:
                return []
            lgx, lgy = snap
        """
        #print(f"new lsx, lsy{lsx, lsy}   lgx, lgy{lgx, lgy}")
        # 若起点=终点，直接返回
        if (lsx, lsy) == (lgx, lgy):
            return [(lsx + x0, lsy + y0)]

        # 4) A* 搜索
        path_local = self._astar(passable, (lsx, lsy), (lgx, lgy))
        # 5) 回到全局像素坐标
        if not path_local:
            print("无法生成有效路径")
            return []
        return [(x + x0, y + y0) for (x, y) in path_local]

    # -------------------- 内部工具：A* --------------------

    def _astar(self,
               passable: List[List[bool]],
               start: Tuple[int, int],
               goal: Tuple[int, int]) -> List[Tuple[int, int]]:
        H = len(passable)
        W = len(passable[0]) if H else 0
        if H == 0 or W == 0:
            return []

        sx, sy = start
        gx, gy = goal

        # 邻接偏移与移动代价
        if self.allow_diag:
            nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
                    (-1, -1, math.sqrt(2)), (1, -1, math.sqrt(2)),
                    (-1, 1, math.sqrt(2)), (1, 1, math.sqrt(2))]
        else:
            nbrs = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0)]

        def heuristic(x: int, y: int) -> float:
            dx = abs(x - gx)
            dy = abs(y - gy)
            if self.allow_diag:
                D = 1.0
                D2 = math.sqrt(2)
                return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)  # octile
            else:
                return dx + dy  # 曼哈顿

        # 打开表（小根堆）: (f, tie, x, y)
        import heapq
        open_heap: List[Tuple[float, int, int, int]] = []
        tie = 0

        g = { (sx, sy): 0.0 }
        parent: Dict[Tuple[int, int], Tuple[int, int]] = {}

        heapq.heappush(open_heap, (heuristic(sx, sy), tie, sx, sy))
        tie += 1

        closed = set()

        while open_heap:
            _, _, x, y = heapq.heappop(open_heap)
            if (x, y) in closed:
                continue
            if (x, y) == (gx, gy):
                return self._reconstruct(parent, (gx, gy), (sx, sy))
            closed.add((x, y))

            for dx, dy, step_cost in nbrs:
                nx, ny = x + dx, y + dy
                if not self._is_inside(nx, ny, W, H):
                    continue
                if not passable[ny][nx]:
                    continue
                # 8 邻接时禁止“拐角穿越”：斜向移动要求两侧正交邻接均可通行
                if self.allow_diag and dx != 0 and dy != 0:
                    if not (passable[y][ny - dy] if False else True):  # 占位，便于阅读
                        pass  # 无效
                    ax, ay = x + dx, y
                    bx, by = x, y + dy
                    if not (passable[ay][ax] and passable[by][bx]):
                        continue

                new_g = g[(x, y)] + step_cost

                # 稍微加入“直线偏好”的微小代价，有助于打破等代价抖动
                # （也可以用父方向向量的微罚项；此处保持简单）
                if (nx, ny) not in g or new_g < g[(nx, ny)]:
                    g[(nx, ny)] = new_g
                    parent[(nx, ny)] = (x, y)
                    f = new_g + heuristic(nx, ny)
                    heapq.heappush(open_heap, (f, tie, nx, ny))
                    tie += 1

        return []

    def simplify_path(self,
                      raw_path: List[Tuple[int, int]],
                      grid: List[List[int]],
                      target_distance_pix: int = 20,
                      min_distance_pix: int = 2,
                      turn_angle_threshold_deg: float = 20.0) -> List[Tuple[int, int]]:
        """
        简化密集路径，减少路径点数量同时确保不穿墙

        Args:
            raw_path: 原始密集路径 [(x1,y1), (x2,y2), ...]
            grid: 栅格地图，用于碰撞检测
            target_distance_pix: 期望的相邻路径点像素距离
            min_distance_pix: 最小允许距离（转弯处）
            turn_angle_threshold_deg: 判定转弯的角度阈值

        Returns:
            简化后的路径点列表
        """
        if len(raw_path) <= 2:
            return raw_path

        simplified = [raw_path[0]]  # 起点必须保留
        current_idx = 0

        while current_idx < len(raw_path) - 1:
            best_next_idx = self._find_best_next_waypoint(
                raw_path, grid, current_idx,
                target_distance_pix, min_distance_pix, turn_angle_threshold_deg
            )

            simplified.append(raw_path[best_next_idx])
            current_idx = best_next_idx

        # 确保终点被包含（可能已经在上面添加了）
        if simplified[-1] != raw_path[-1]:
            simplified.append(raw_path[-1])

        return simplified

    def _find_best_next_waypoint(self,
                                 raw_path: List[Tuple[int, int]],
                                 grid: List[List[int]],
                                 current_idx: int,
                                 target_dist: int,
                                 min_dist: int,
                                 turn_threshold_deg: float) -> int:
        """
        从当前点开始，找到最佳的下一个路径点

        优先级：
        1. 尽可能接近target_distance的点
        2. 转弯处允许使用较短距离
        3. 确保连线不穿墙
        """
        current_pos = raw_path[current_idx]

        # 候选点：从当前点向后搜索
        best_idx = current_idx + 1  # 最差情况下选择下一个点
        best_score = float('inf')

        max_search_range = min(current_idx + target_dist * 2, len(raw_path) - 1)

        for candidate_idx in range(current_idx + 1, max_search_range + 1):
            candidate_pos = raw_path[candidate_idx]
            distance = self._pixel_distance(current_pos, candidate_pos)

            # 距离太小，跳过（除非是最后一个点）
            if distance < min_dist and candidate_idx < len(raw_path) - 1:
                continue

            # 检查是否穿墙
            if not self._is_line_clear(grid, current_pos, candidate_pos):
                continue

            # 计算得分（距离越接近target_dist越好）
            distance_score = abs(distance - target_dist)

            # 检查是否为转弯点，转弯点允许较短距离
            is_turn = self._is_turning_point(raw_path, candidate_idx, turn_threshold_deg)
            if is_turn and distance >= min_dist:
                distance_score *= 0.7  # 转弯点得分加权

            if distance_score < best_score:
                best_score = distance_score
                best_idx = candidate_idx

            # 如果找到了接近目标距离的点，可以早期退出
            if distance >= target_dist * 0.9 and distance <= target_dist * 1.1:
                break

        return best_idx

    def _pixel_distance(self, pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """计算两个像素点之间的欧几里德距离"""
        return math.sqrt((pos1[0] - pos2[0]) ** 2 + (pos1[1] - pos2[1]) ** 2)

    def _is_line_clear(self, grid: List[List[int]], start: Tuple[int, int], end: Tuple[int, int]) -> bool:
        """
        使用Bresenham算法检查两点间直线是否穿过障碍物

        Returns:
            True: 路径清晰，无障碍物
            False: 路径被障碍物阻挡
        """
        H = len(grid)
        W = len(grid[0]) if H > 0 else 0

        x0, y0 = start
        x1, y1 = end

        # Bresenham直线算法
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        if dx == 0 and dy == 0:
            return True

        x, y = x0, y0
        x_inc = 1 if x1 > x0 else -1
        y_inc = 1 if y1 > y0 else -1

        if dx > dy:
            error = dx / 2
            while x != x1:
                # 检查当前点是否为障碍物
                if not (0 <= x < W and 0 <= y < H) or grid[y][x] <= self.obst_v:
                    return False

                error -= dy
                if error < 0:
                    y += y_inc
                    error += dx
                x += x_inc
        else:
            error = dy / 2
            while y != y1:
                # 检查当前点是否为障碍物
                if not (0 <= x < W and 0 <= y < H) or grid[y][x] <= self.obst_v:
                    return False

                error -= dx
                if error < 0:
                    x += x_inc
                    error += dy
                y += y_inc

        # 检查终点
        if not (0 <= x1 < W and 0 <= y1 < H) or grid[y1][x1] <= self.obst_v:
            return False

        return True

    def _is_turning_point(self, path: List[Tuple[int, int]], idx: int, angle_threshold: float) -> bool:
        """
        检查某个路径点是否为转弯点

        通过计算前后方向向量的角度变化来判断
        """
        if idx < 2 or idx >= len(path) - 1:
            return False

        # 获取三个连续点
        p1 = path[idx - 2]
        p2 = path[idx]
        p3 = path[idx + 2] if idx + 2 < len(path) else path[-1]

        # 计算两个方向向量
        v1 = (p2[0] - p1[0], p2[1] - p1[1])
        v2 = (p3[0] - p2[0], p3[1] - p2[1])

        # 计算角度变化
        angle_change = self._vector_angle_diff(v1, v2)

        return abs(angle_change) > angle_threshold

    def _vector_angle_diff(self, v1: Tuple[float, float], v2: Tuple[float, float]) -> float:
        """计算两个向量之间的角度差（度）"""
        import math

        # 防止零向量
        len1 = math.sqrt(v1[0] ** 2 + v1[1] ** 2)
        len2 = math.sqrt(v2[0] ** 2 + v2[1] ** 2)

        if len1 < 1e-6 or len2 < 1e-6:
            return 0.0

        # 计算角度
        angle1 = math.atan2(v1[1], v1[0])
        angle2 = math.atan2(v2[1], v2[0])

        # 角度差，归一化到[-180, 180]
        diff = math.degrees(angle2 - angle1)
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360

        return diff

    # 修改原有的plan_path方法，添加路径简化选项
    def plan_path(self,
                  grid: List[List[int]],
                  start_pix: Tuple[int, int],
                  goal_pix: Tuple[int, int],
                  maze_rect: Optional[Tuple[int, int, int, int]],
                  simplify: bool = True,
                  target_distance_pix: int = 40) -> List[Tuple[int, int]]:
        """
        规划路径，可选择是否简化

        Args:
            simplify: 是否对路径进行简化
            target_distance_pix: 简化时的目标像素距离
        """
        # 调用原有的A*算法
        raw_path = self._plan_path_original(grid, start_pix, goal_pix, maze_rect)

        if not raw_path or not simplify:
            print("not raw path")
            return raw_path

        # 简化路径
        simplified_path = self.simplify_path(
            raw_path, grid,
            target_distance_pix=target_distance_pix,
            min_distance_pix=max(2, target_distance_pix // 3)
        )

        print(f"路径简化: {len(raw_path)} -> {len(simplified_path)} 点 "
              f"(压缩率: {len(simplified_path) / len(raw_path):.2%})")

        return simplified_path

    def _plan_path_original(self,
                            grid: List[List[int]],
                            start_pix: Tuple[int, int],
                            goal_pix: Tuple[int, int],
                            maze_rect: Optional[Tuple[int, int, int, int]]) -> List[Tuple[int, int]]:
        """
        原始的A*路径规划方法（重命名原来的plan_path）
        """
        # 这里是原来plan_path方法的所有代码
        # [保持原有的A*实现不变]
        if not grid or not grid[0]:
            print("121212")
            return []

        H = len(grid)
        W = len(grid[0])

        # 1) 约束 ROI
        if maze_rect is None:
            x0, y0, x1, y1 = 0, 0, W - 1, H - 1
        else:
            x0, y0, x1, y1 = maze_rect
            x0 = max(0, min(x0, W - 1))
            x1 = max(0, min(x1, W - 1))
            y0 = max(0, min(y0, H - 1))
            y1 = max(0, min(y1, H - 1))
            if x1 < x0 or y1 < y0:
                print("232323")
                return []

        # 2) 在 ROI 内构建"可通行掩模"并对障碍做膨胀
        passable, roi_W, roi_H = self._build_passable_with_inflation(grid, (x0, y0, x1, y1))

        # 3) 将 start/goal 转到 ROI 局部坐标，并吸附到最近可通行点
        sx, sy = start_pix
        gx, gy = goal_pix
        if not (x0 <= sx <= x1 and y0 <= sy <= y1 and x0 <= gx <= x1 and y0 <= gy <= y1):
            print(f"gx,gy{gx,gy}")
            print("不在roi内")
            return []

        lsx, lsy = sx - x0, sy - y0
        lgx, lgy = gx - x0, gy - y0

        if not self._is_inside(lsx, lsy, roi_W, roi_H) or not self._is_inside(lgx, lgy, roi_W, roi_H):
            print("not inside")
            return []

        if not passable[lsy][lsx]:
            print("小车当前点不在passable中")
            snap = self._snap_to_nearest_passable(passable, lsx, lsy)
            if snap is None:
                print("小车当前点无法snap")
                return []
            lsx, lsy = snap

        if not passable[lgy][lgx]:
            print("目标点不在passable中")
            snap = self._snap_to_nearest_passable(passable, lgx, lgy)
            if snap is None:
                print("目标点无法snap")
                return []
            lgx, lgy = snap

        # 若起点=终点，直接返回
        if (lsx, lsy) == (lgx, lgy):
            return [(lsx + x0, lsy + y0)]

        # 4) A* 搜索
        path_local = self._astar(passable, (lsx, lsy), (lgx, lgy))
        # 5) 回到全局像素坐标
        if not path_local:
            print("a*无法生成路径")
            return []
        return [(x + x0, y + y0) for (x, y) in path_local]

    @staticmethod
    def _reconstruct(parent: Dict[Tuple[int, int], Tuple[int, int]],
                     goal: Tuple[int, int],
                     start: Tuple[int, int]) -> List[Tuple[int, int]]:
        path = [goal]
        cur = goal
        while cur != start:
            cur = parent.get(cur)
            if cur is None:
                return []  # 防御性：不应发生
            path.append(cur)
        path.reverse()
        return path

    # -------------------- 内部工具：ROI 可通行掩模 & 膨胀 --------------------

    def _build_passable_with_inflation(self,
                                       grid: List[List[int]],
                                       roi: Tuple[int, int, int, int]
                                       ) -> Tuple[List[List[bool]], int, int]:
        """
        在 roi 内生成 passable[y][x]（局部坐标），并对障碍做膨胀。
        - 仅把 >= free_thresh 的格子设为 True
        - 对“障碍==0”做半径 inflate_r 的圆形膨胀；未知不膨胀但不可通行
        """
        x0, y0, x1, y1 = roi
        H = len(grid)
        W = len(grid[0]) if H else 0
        roi_W = x1 - x0 + 1
        roi_H = y1 - y0 + 1

        # 初始 passable
        passable: List[List[bool]] = [[False] * roi_W for _ in range(roi_H)]
        obstacles: List[Tuple[int, int]] = []

        for y in range(y0, y1 + 1):
            row = grid[y]
            py = y - y0
            for x in range(x0, x1 + 1):
                v = row[x]
                px = x - x0
                if v >= self.free_th:
                    passable[py][px] = True
                if v <= self.obst_v:
                    obstacles.append((px, py))

        # 膨胀：把障碍周围 r 内的 passable 置 False
        r = self.inflate_r
        print(f"self.inflate_r{self.inflate_r}")
        if r > 0 and obstacles:
            rr = r * r
            for (ox, oy) in obstacles:
                # 在局部小窗口内遍历，避免全图遍历
                x_min = max(0, ox - r)
                x_max = min(roi_W - 1, ox + r)
                y_min = max(0, oy - r)
                y_max = min(roi_H - 1, oy + r)
                for py in range(y_min, y_max + 1):
                    dy = py - oy
                    dy2 = dy * dy
                    for px in range(x_min, x_max + 1):
                        dx = px - ox
                        if dx * dx + dy2 <= rr:
                            passable[py][px] = False

        return passable, roi_W, roi_H

    # -------------------- 内部工具：吸附到最近可通行栅格 --------------------

    def _snap_to_nearest_passable(self,
                                  passable: List[List[bool]],
                                  sx: int, sy: int,
                                  search_step: int = 7) -> Optional[Tuple[int, int]]:
        """
        若 (sx,sy) 不可通行，则在 max_snap_r 的切比雪夫邻域内寻找最近可通行格（BFS 环扩展）。
        可通过 search_step 控制邻域步长，从而改变吸附的“粒度”。
        返回局部坐标 (x,y) 或 None。
        """
        H = len(passable)
        W = len(passable[0]) if H else 0
        if self._is_inside(sx, sy, W, H) and passable[sy][sx]:
            return (sx, sy)

        from collections import deque
        q = deque()
        seen = set()
        q.append((sx, sy, 0))
        seen.add((sx, sy))

        # 根据 search_step 生成多尺度邻域
        dirs = []
        for dy in range(-search_step, search_step + 1):
            for dx in range(-search_step, search_step + 1):
                if dx == 0 and dy == 0:
                    continue
                # 仍保持8邻接原则，但允许跳步
                if max(abs(dx), abs(dy)) <= search_step:
                    dirs.append((dx, dy))

        while q:
            x, y, d = q.popleft()
            if d > self.max_snap_r:
                break
            if self._is_inside(x, y, W, H) and passable[y][x]:
                return (x, y)
            nd = d + 1
            for dx, dy in dirs:
                nx, ny = x + dx, y + dy
                if not self._is_inside(nx, ny, W, H):
                    continue
                if (nx, ny) in seen:
                    continue
                seen.add((nx, ny))
                q.append((nx, ny, nd))

        return None

    @staticmethod
    def _is_inside(x: int, y: int, W: int, H: int) -> bool:
        return 0 <= x < W and 0 <= y < H


class MotionPrimitives:
    """
    将“下一局部目标点/角”（世界坐标系，m/deg）转为一段或多段“左右轮**累计**编码器计数 + 时长”的命令序列。
    约定：Planner 维护“累计计数”的当前值（由上层回传同步），这里只负责“增量->目标累计”的换算。
    """
    def __init__(self, geom: WheelGeom, vel: VelocityLimits):
        self.geom = geom
        self.vel = vel

    # --- 几个基础换算工具（供你实现时使用） ---
    def meters_to_ticks(self, dist_m: float) -> int:
        """线位移 -> 两轮等距旋转（前进/后退）的编码器ticks增量（不含累计）。"""
        # ticks = (弧度 = 距离/半径) / (2π) * ticks_per_cycle
        circ_mm = 2.0 * math.pi * self.geom.wheel_radius_mm
        rev = (dist_m * 1000.0) / circ_mm
        return int(round(rev * self.geom.ticks_per_cycle))

    def deg_to_ticks_diff(self, yaw_deg: float) -> int:
        """纯自转时，每个车轮的**相反方向**转动的ticks增量幅值（不含累计）。"""
        # 自转角度θ对应轮缘位移： s = θ(rad) * 半轴长(mm)
        s_mm = abs(math.radians(yaw_deg)) * self.geom.half_axle_len_mm
        rev = s_mm / (2.0 * math.pi * self.geom.wheel_radius_mm)
        return int(round(rev * self.geom.ticks_per_cycle))

    def segment_time_ms(self, lin_dist_m: float = 0.0, yaw_deg: float = 0.0) -> int:
        """根据限速估计本段命令时长，裁剪到[min,max]区间，返回ms。"""
        t_lin = abs(lin_dist_m) / max(1e-6, self.vel.max_lin_mps) if abs(lin_dist_m) > 1e-6 else 0.0
        t_yaw = abs(yaw_deg) / max(1e-6, self.vel.max_ang_dps) if abs(yaw_deg) > 1e-6 else 0.0
        t = max(t_lin, t_yaw, self.vel.min_segment_time_s)
        t = min(t, self.vel.max_segment_time_s)
        return int(round(t * 1000.0))

    # --- 原始动作原语（先转再走，再微调） ---
    def turn_in_place(self, curr_cum: Tuple[int,int], yaw_deg: float, goal_pose: Pose,
                      tol: CmdTolerance) -> EncoderTarget:
        """
        原地转向到目标朝向附近。生成一条“累计ticks+时长”的命令。
        - yaw_deg>0 左轮后退/右轮前进；yaw_deg<0 相反。
        """
        d_ticks = self.deg_to_ticks_diff(yaw_deg)
        dl = curr_cum[0] - d_ticks if yaw_deg > 0 else curr_cum[0] + d_ticks
        dr = curr_cum[1] + d_ticks if yaw_deg > 0 else curr_cum[1] - d_ticks
        dur = self.segment_time_ms(yaw_deg=abs(yaw_deg))
        return EncoderTarget(dl, dr, dur, goal_pose, tol)

    def drive_straight(self, curr_cum: Tuple[int,int], dist_m: float, goal_pose: Pose,
                       tol: CmdTolerance) -> EncoderTarget:
        """
        直线段：两轮同向同量。
        """
        d_ticks = self.meters_to_ticks(dist_m)
        dl = curr_cum[0] + d_ticks
        dr = curr_cum[1] + d_ticks
        dur = self.segment_time_ms(lin_dist_m=abs(dist_m))
        return EncoderTarget(dl, dr, dur, goal_pose, tol)

    # --- 高层：给定“下一路标”（世界坐标），切成“先转-直行-微调”的一到三条命令 ---
    def waypoint_to_commands(self,
                             curr_pose: Pose,
                             waypoint: Pose,
                             curr_cum: Tuple[int,int],
                             tol: CmdTolerance) -> List[EncoderTarget]:
        """
        将“从 curr_pose 到 waypoint”的动作离散为最多三段：
        1) 原地转向：把车头对准路标
        2) 直行至路标邻域
        3) 角度微调（可选）
        注意：这里只做**规划**，返回的累计ticks=当前累计+本段增量，
             上层执行后应用回传把“当前累计”与“实际位姿”更新，再决定是否还需“微调重发”。
        """
        dx = waypoint.x_m - curr_pose.x_m
        dy = waypoint.y_m - curr_pose.y_m
        dist = math.hypot(dx, dy)
        tgt_yaw = math.degrees(math.atan2(dy, dx))
        # 归一化：
        def wrap(a): return (a + 180.0) % 360.0 - 180.0
        d_yaw1 = wrap(tgt_yaw - curr_pose.theta_deg)
        d_yaw2 = wrap(waypoint.theta_deg - tgt_yaw)

        cmds: List[EncoderTarget] = []
        # 1) 转向
        if abs(d_yaw1) > 1e-3:
            cmds.append(self.turn_in_place(curr_cum, d_yaw1,
                                           Pose(curr_pose.x_m, curr_pose.y_m, tgt_yaw), tol))
            # 预估执行后累计（供2段起点使用）
            curr_cum = (cmds[-1].cum_left_ticks, cmds[-1].cum_right_ticks)

        # 2) 直行
        if dist > 1e-3:
            cmds.append(self.drive_straight(curr_cum, dist,
                                            Pose(waypoint.x_m, waypoint.y_m, tgt_yaw), tol))
            curr_cum = (cmds[-1].cum_left_ticks, cmds[-1].cum_right_ticks)

        # 3) 航向微调
        if abs(d_yaw2) > 1e-3:
            cmds.append(self.turn_in_place(curr_cum, d_yaw2,
                                           Pose(waypoint.x_m, waypoint.y_m, waypoint.theta_deg), tol))
        return cmds


# ======================================================================================
#                                  主 Planner（状态机）
# ======================================================================================

class Planner:
    """
    统一的路径规划与任务管理器：
    - 输入：SLAM栅格(ListGridMap)、当前车姿态（米/度，来自SLAM/里程融合）、当前**累计**编码器计数
    - 输出：一段或多段 EncoderTarget（每段= 左/右轮累计ticks + 执行时长），并在内部维护“下一局部目标点”
    - 策略：EXPLORE -> RETURN_HOME -> GO_TO_EXIT -> DONE
    - 探索完成判据：在估计的“迷宫有效矩形”内，所有“可达前沿”都为空（或只剩不可达/低收益），且已发现出口
    """
    def __init__(self,
                 gridmap: ListGridMap,
                 wheel_geom: WheelGeom,
                 vel_limits: VelocityLimits = VelocityLimits(),
                 tol: CmdTolerance = CmdTolerance(),
                 goal_world: Optional[Tuple[float, float]] = None,
                 ctol: CTolerance = CTolerance()):
        self.map = gridmap
        self.phase = MissionPhase.EXPLORE

        # 坐标转换器（新增）
        self.coord_converter = CoordinateConverter(
            map_size_pixels=gridmap.map_size_pixels,
            map_size_meters=gridmap.map_scale_meters_per_pixel * gridmap.map_size_pixels
        )

        # 关键子模块
        self.rectifier = MapRectifier()
        self.explorer = OptimizedFrontierExplorer()
        self.goal_checker = GoalReachabilityChecker()
        self.global_planner = GlobalPathPlanner()
        self.motion = MotionPrimitives(wheel_geom, vel_limits)
        self.tol = tol

        # 终点管理（新增）
        self.goal_world_m = goal_world  # 终点世界坐标（米）
        self.goal_pix: Optional[Tuple[int, int]] = None  # 终点像素坐标

        if goal_world:
            # 转换终点坐标
            self.goal_pix = self.coord_converter.world_to_pixel(goal_world[0], goal_world[1])
            print(f"终点设置: 世界坐标{goal_world} -> 像素坐标{self.goal_pix}")

        # 关键里程碑
        self.start_pose_world: Optional[Pose] = None   # 起点（世界系）
        self.exit_pose_world: Optional[Pose] = None    # 终点（世界系）
        self.exit_pose_pix = None #终点（像素系系）
        self.goal_path_pix: List[Tuple[int,int]] = []  # 目前在执行的像素路径
        self.current_waypoint: Optional[Pose] = None   # 即将驱动到的世界系路标（用于产生编码器命令）
        self.best_frontier = [] #按照分数从高往低排列的前沿点列表
        self.goal_frontier = None #当前选择的最优前沿点

        # 统计/可视化
        self.frontier_cache: List[Tuple[int,int]] = []
        self.maze_rect: Optional[Tuple[int,int,int,int]] = None
        self.note: str = ""

        #原地摇摆建图
        self._wiggle_toggle = 2  # 在左/右之间切换
        self._wiggle_yaw_deg = 1  # 每次原地转动角度（可调 10~30）

        self.spatially_sampled = None
        self.repick = False
        self.ctol = ctol
    # ------------------ 生命周期入口 ------------------

    def set_start_pose(self, pose_world: Pose) -> None:
        """在比赛起点初始化时调用一次，记录起点。"""
        self.start_pose_world = pose_world

    def set_exit_pose(self, pose_world: Pose) -> None:
        """如果你有外部“终点”提示，也可以直接注入；否则由 ExitDetector 在探索中发现。"""
        self.exit_pose_world = pose_world

    def waypoint_to_turn_and_go(self,
                                curr_pose: Pose,
                                waypoint: Pose,
                                tol: CTolerance
                                ) -> Tuple[float, float]:
        """
        生成“(转角deg, 直行m)”的命令序列，遵循：
          - 角度：逆时针为正、顺时针为负（单位：度）
          - 距离：单位：米
        策略：先将车头对准路标，再直行到路标邻域（只要位置进入 pos_tol_m 即可）。
        若起点已在路标邻域，则仅返回角度微调（可按需要保留或省略）。
        """

        def wrap_deg(a: float) -> float:
            # 归一化到 (-180, +180]
            w = (a + 180.0) % 360.0 - 180.0
            return 180.0 if w == -180.0 else w

        # 位置差与指向角
        dx = waypoint.x_m - curr_pose.x_m
        dy = waypoint.y_m - curr_pose.y_m
        dist = math.hypot(dx, dy)
        tgt_yaw = math.degrees(math.atan2(dy, dx))  # 面向路标的期望朝向（度，逆时针为正）

        cmds: List[Tuple[float, float]] = []

        if dist <= tol.pos_tol_m:
            print("%%%只做角度微调&&&&")
            # 已在路标邻域：可以只做角度微调（按需要保留/注释）
            d_yaw = wrap_deg(waypoint.theta_deg - curr_pose.theta_deg)
            if abs(d_yaw) < 3 :
                return 0 ,0
            else :
                return d_yaw, 0


        # 第一步：把车头转到指向路标（顺时针为负、逆时针为正）
        d_yaw1 = wrap_deg(tgt_yaw - curr_pose.theta_deg)
        if abs(d_yaw1) < 2:
            return 0, dist
        else:
            return d_yaw1, dist

        # 如需在到达后再对齐到 waypoint 的目标朝向，可在上层检测到“已到达位置邻域”后，
        # 再调用本函数一次，此时只会返回角度微调段（见前面的分支）。

    def swing_in_place(self) :
        """
        在无路标目标时生成一个左右摇摆的“转角命令”。
        - 原地旋转，保持SLAM更新。
        - 每次返回一个 [(turn_deg, distance_m)]。
        - 角度单位：度，逆时针为正。
        """
        # 可配置参数
        swing_angle_deg = 15.0  # 每次转动 ±15度
        distance_m = 0.05  # 原地旋转不直行

        # 如果还没有状态，就初始化一个标志
        if not hasattr(self, "_swing_dir"):
            self._swing_dir = 1  # 1 表示先左转（逆时针）

        # 根据方向生成指令
        turn_deg = swing_angle_deg * self._swing_dir

        # 翻转方向（下一次反向转）
        self._swing_dir *= -1

        print(f"[Planner] 摆头命令：{turn_deg:+.1f}°")

        return turn_deg, distance_m

    # ------------------ 规划主循环接口 ------------------

    def plan_next(self,
                  curr_pose_world: Pose) -> Tuple[float,float, PlannerStateReport]:
        """
        “一步规划”：基于当前SLAM地图与位姿，决定下一批命令（1~3段），同时返回状态报告。
        上层流程建议：
          1) 调用 plan_next() 得到 cmds；
          2) 逐条下发给底层（或仿真），等待时长结束/里程更新；
          3) 若回传的位置没进容差 -> 继续“小步微调”：再次调用 plan_next()（不换新路标，只补偿）。
        """
        update_start_time = time.time()
        self._update_derived_state()
        update_end_time = time.time()-update_start_time
        print(f"update_time{update_end_time}")


        ensure_waypoint_start_time = time.time()
        # 状态机切换 & 目标生成
        self._ensure_waypoint(curr_pose_world)
        end_time = time.time()-ensure_waypoint_start_time
        print(f"ensure_waypoint_time{end_time}")

        commands = None
        #self.current_waypoint = Pose(curr_pose_world.x_m+0.05,curr_pose_world.y_m,curr_pose_world.theta_deg)
        if self.current_waypoint:
            commands = self.waypoint_to_turn_and_go(curr_pose_world,self.current_waypoint,self.ctol)

        else:
            print("正在摇摆")
            # 没有路标 -> 做原地摆头，保证主循环有命令可执行、SLAM可持续更新
            commands = self.swing_in_place()

        # 生成报告
        report = PlannerStateReport(
            phase=self.phase,
            curr_pose=curr_pose_world,
            next_waypoint=self.current_waypoint,
            frontier_count=len(self.frontier_cache),
            known_maze_rect=self.maze_rect,
            has_found_exit=(self.exit_pose_world is not None),
            path_len_nodes=len(self.goal_path_pix),
            note=self.note,
            frontier_points=self.frontier_cache.copy(),
            best_frontier_point=self.goal_frontier,
            Spatially_sampled = self.explorer.SPATIALLY,
            exit_pose=self.goal_pix

        )
        turn_deg, distance_m = commands
        #print(turn_deg,distance_m, report)
        return turn_deg,distance_m, report

    def _bootstrap_scan(self,
                        curr_pose_world: Pose,
                        curr_cum_ticks: Tuple[int, int]) -> List[EncoderTarget]:
        """当当前无路标/无路径时：原地小幅左右摆头，多扫几帧让SLAM更快成图。"""
        yaw = self._wiggle_toggle * self._wiggle_yaw_deg
        self._wiggle_toggle *= 1  # 下次换方向
        # 目标姿态仅用于显示：位置不变，朝向临时加 yaw
        tgt = Pose(curr_pose_world.x_m, curr_pose_world.y_m,
                   curr_pose_world.theta_deg + yaw)
        cmd = self.motion.turn_in_place(curr_cum_ticks, yaw, tgt, self.tol)
        self.note = "Bootstrap scan: wiggle to enrich map"
        return [cmd]

    # ------------------ 内部：每次规划前刷新派生信息 ------------------

    def _update_derived_state(self) -> None:
        """刷新：迷宫矩形估计、前沿列表、是否发现出口、是否切换阶段。"""
        # 1) 更新“迷宫有效矩形”
        cx_pix,cy_pix,_ = self.map.GetCarPose()
        self.maze_rect = self.rectifier.update_from_grid(self.map.grid,
                                                         ensure_contains=(int(round(cx_pix)), int(round(cy_pix))))
        print(f"maze_rect{self.maze_rect}")

        # 2) 若在探索阶段，则提取前沿与检测出口
        if self.phase == MissionPhase.EXPLORE:
            self.frontier_cache = self.explorer.detect_frontiers(self.map.grid, self.maze_rect)
            self.note = "exploring the map......"

            # 【新逻辑】检查终点是否可达
            if self.goal_pix is not None:
                goal_reachable = self.goal_checker.is_goal_reachable(
                    self.map.grid,
                    (cx_pix, cy_pix),
                    self.goal_pix,
                    self.maze_rect
                )

                if goal_reachable:
                    # 终点可达，直接进入返回阶段
                    print("=" * 50)
                    print("终点已可达！切换到GO_TO_EXIT阶段")
                    print("=" * 50)
                    self.phase = MissionPhase.GO_TO_EXIT
                    self.goal_path_pix.clear()
                    self.current_waypoint = None
                    self.note = "Goal reachable -> GO_TO_EXIT"
                    return

            self.note = "Exploring towards goal..."

    # ------------------ 内部：选择/生成“下一个路标” ------------------

    def _ensure_waypoint(self, curr_pose_world: Pose) -> None:
        """
        如果当前没有路标，或路标已到达/失效，则生成新路标：
        - EXPLORE：选前沿 -> 全局A*到该前沿 -> 取路径下一小段转换为局部waypoint
        - RETURN_HOME：A*到起点
        - GO_TO_EXIT：A*从起点到终点
        """
        # 1) 若没有起点记录，则初始化
        if self.start_pose_world is None:
            self.start_pose_world = Pose(curr_pose_world.x_m, curr_pose_world.y_m, curr_pose_world.theta_deg)

        # 2) 若已有路标且仍未到达（上层通过重复调用 plan_next() 会微调），则保持不变
        if self.current_waypoint and not self._reached(curr_pose_world, self.current_waypoint, self.tol):
            print("未到达目标点附近")
            return

        # 3) 生成/刷新路径与路标
        if self.phase == MissionPhase.EXPLORE:
            cx_pix,cy_pix,_= self.map.GetCarPose()
            # 若已到达最优前沿点附近或者path长度小于2 就重新找最优前沿点并规划路径
            if self._near_without_deg(cx_pix,cy_pix,self.goal_frontier) or len(self.goal_path_pix)<=3:
                print("正在生成最优前沿点并规划路径")
                self._gen_waypoint_explore(curr_pose_world)
            else:
                self.current_waypoint = self._pop_next_waypoint(curr_pose_world)

        elif self.phase == MissionPhase.RETURN_HOME:
            if self._plan_or_refresh_path_to_world(curr_pose_world, self.start_pose_world,False):
                self.current_waypoint = self._pop_next_waypoint(curr_pose_world)
            else:
                # 到家或路径空
                if self._near(curr_pose_world, self.start_pose_world, self.tol):
                    # 下一阶段：从起点 -> 终点
                    if self.exit_pose_world:
                        self.phase = MissionPhase.GO_TO_EXIT
                        self.goal_path_pix.clear()
                        self.current_waypoint = None
                        self.note = "At start -> GO_TO_EXIT"
                    else:
                        self.note = "Waiting for exit_pose_world"
                else:
                    self.note = "No path to home (check map/inflation)"

        elif self.phase == MissionPhase.GO_TO_EXIT:
            # 从“起点”发起最短路（你也可以选择从“当前位置”直接到exit再回到起点后走最短路）
            goal_world_pose = Pose(self.goal_world_m[0],self.goal_world_m[1],0)
            if self._plan_or_refresh_path_to_world(curr_pose_world, goal_world_pose,True):
                self.current_waypoint = self._pop_next_waypoint(curr_pose_world)
            else:
                # 若路径已全部消耗/到终点
                cx_pix, cy_pix,_ = self.map.GetCarPose()

                if self._near_without_deg(cx_pix, cy_pix, self.goal_pix):
                    self.phase = MissionPhase.RETURN_HOME
                    self.current_waypoint = None
                    self.note = "Return home"

        elif self.phase == MissionPhase.DONE:
            self.current_waypoint = None
            self.note = "All done."

    # ------------------ 内部：阶段细节 ------------------

    def _gen_waypoint_explore(self, curr_pose_world: Pose) -> None:
        """挑选前沿并规划到前沿，再从路径中取一小段转为waypoint。"""
        if not self.frontier_cache:
            self.note = "No frontier candidates"
            print("not 没有frontier_cache")
            self.current_waypoint = None
            self.best_frontier = []
            return
        if len(self.frontier_cache)==0:
            print("没有frontier_cache")
            self.best_frontier = []
        # 当前像素
        cx_pix, cy_pix, _ = self.map.GetCarPose()
        print(f"current position{cx_pix},{cy_pix}")
        if len(self.best_frontier)<1:
            self.repick = False

        if self.repick == False:
            #重新生成一批前沿点按照分数从大到小的排序
            goal_pix_s  = self.explorer.pick_goal((cx_pix, cy_pix), self.frontier_cache,self.map.grid,goal_pix=self.goal_pix,maze_rect=self.maze_rect)
            self.best_frontier = goal_pix_s
            if len(goal_pix_s)<=0:
                return
            goal_pix = self.best_frontier.pop(0)
            self.goal_frontier = goal_pix
            self.repick = True
        else:
            goal_pix = self.best_frontier.pop(0)
            self.goal_frontier = goal_pix


        print(f"goal_pix{goal_pix}")
        if goal_pix is None:
            self.note = "No valid frontier after filtering"
            self.current_waypoint = None
            return

        # A* 到前沿
        self.goal_path_pix = self.global_planner.plan_path(self.map.grid,
                                                           (int(round(cx_pix)), int(round(cy_pix))),
                                                           goal_pix, self.maze_rect)
        print(f"goal_path_pix{self.goal_path_pix}")
        if len(self.goal_path_pix) < 2:
            self.note = "Path to frontier empty"
            self.current_waypoint = None
            self.repick = True
            return
        else:
            self.repick = False

        # 取“下一跳”（或按固定步长抽点）为局部waypoint
        self.current_waypoint = self._pop_next_waypoint(curr_pose_world)

    def _plan_or_refresh_path_to_world(self, curr_pose_world: Pose, goal_world: Pose, go_to_exiit:bool) -> bool:
        """从当前像素到世界坐标goal的像素点规划；有路径则缓存。"""
        cx_pix, cy_pix, _ = self.map.GetCarPose()
        if go_to_exiit:
            gx_pix, gy_pix = self.coord_converter.world_to_pixel(goal_world.x_m,goal_world.y_m)
        else:
            gx_pix, gy_pix = self.map.m2pix(goal_world.x_m,goal_world.y_m)
        print(f"plan{gx_pix,gy_pix}")
        self.goal_path_pix = self.global_planner.plan_path(self.map.grid,
                                                           (int(round(cx_pix)), int(round(cy_pix))),
                                                           (int(round(gx_pix)), int(round(gy_pix))),
                                                           self.maze_rect)
        print(f"len of goal path pix{self.goal_path_pix}")
        return len(self.goal_path_pix) >= 3

    def _pop_next_waypoint(self, curr_pose_world: Pose) -> Optional[Pose]:
        """
        将像素路径的下一节点转成“世界局部路标”，建议：
        - 朝向 = 指向下一节点的航向
        - 若路径只剩终点节点，可把航向设为“面向下一个走廊方向”或与起点/终点约定朝向
        """
        if len(self.goal_path_pix) < 2:
            return None

        nxt = self.goal_path_pix.pop(1)
        print(f"*(*(*(nxt{nxt}*(*(")
        return self._pixnode_to_local_waypoint(curr_pose_world, nxt)

    def _pixnode_to_local_waypoint(self, curr_pose_world: Pose, node_pix: Tuple[int,int]) -> Pose:
        """像素节点 -> 世界路标；航向指向该节点。"""
        x_m, y_m = self.map.pix2m(node_pix[0], node_pix[1])
        dx, dy = x_m - curr_pose_world.x_m, y_m - curr_pose_world.y_m
        yaw = math.degrees(math.atan2(dy, dx))
        print(f"()()(curr+pose_world{curr_pose_world},nxt{x_m,y_m,yaw}()()")
        return Pose(x_m, y_m, yaw)

    # ------------------ 内部：容差判断 ------------------

    @staticmethod
    def _near(a: Pose, b: Pose, tol: CmdTolerance) -> bool:
        print(f"math.hypot(a.x_m - b.x_m, a.y_m - b.y_m){math.hypot(a.x_m - b.x_m, a.y_m - b.y_m)}")
        print(f"abs(((a.theta_deg - b.theta_deg + 180) % 360) - 180){abs(((a.theta_deg - b.theta_deg + 180) % 360) - 180)}")
        return (math.hypot(a.x_m - b.x_m, a.y_m - b.y_m) <= tol.pos_tol_m)

    @staticmethod
    def _reached(curr: Pose, goal: Pose, tol: CmdTolerance) -> bool:
        """到达路标（位置+角度）"""
        print(f"reached检查curr{curr},   goal{goal}")
        return Planner._near(curr, goal, tol)

    @staticmethod
    def _near_without_deg(cur_x_pix,cur_y_pix,best_frontier,max_distance=20):
        if best_frontier is None:
            return False
        goal_x_pix = best_frontier[0]
        goal_y_pix = best_frontier[1]
        distance = math.sqrt((cur_x_pix-goal_x_pix)**2+(cur_y_pix-goal_y_pix)**2)
        return distance<max_distance
