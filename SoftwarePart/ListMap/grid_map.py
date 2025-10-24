from typing import List, Tuple, Set
import numpy as np
from sim.ListMap.draw_map import MapDrawer
from scipy import ndimage
import time
FREE_THRESH = 230
OBSTACLE_THRESH = 60


class ListGridMap:

    def __init__(self, MAP_SIZE_PIXELS, MAP_SIZE_METERS):
        self.w = MAP_SIZE_PIXELS
        self.h = MAP_SIZE_PIXELS
        self.map_size_pixels = MAP_SIZE_PIXELS

        # 主显示地图 grid[y][x]
        self.grid: List[List[int]] = [[128 for _ in range(self.w)] for _ in range(self.h)]

        # === 历史一致性投票系统 ===
        # 障碍物投票计数：每次SLAM输出障碍物时+1
        self.obstacle_votes: np.ndarray = np.zeros((self.h, self.w), dtype=np.int16)

        # 自由空间投票计数：每次SLAM输出自由空间时+1
        self.free_votes: np.ndarray = np.zeros((self.h, self.w), dtype=np.int16)

        # 永久锁定地图：一旦锁定就不再改变
        self.locked_map: np.ndarray = np.full((self.h, self.w), 128, dtype=np.uint8)  # 128=未锁定

        # 锁定状态标记：True表示该像素已被永久锁定
        self.is_locked: np.ndarray = np.zeros((self.h, self.w), dtype=bool)

        # 投票参数
        self.vote_params = {
            'obstacle_lock_threshold': 4,  # 障碍物锁定需要的投票数
            'free_lock_threshold': 15,  # 自由空间锁定需要的投票数
            'conflict_resolve_ratio': 1,  # 冲突解决比例：一方票数需要是另一方的几倍才能获胜
            'noise_filter_threshold': 2,  # 噪声过滤：低于此票数的不参与决策
        }

        self.map_scale_meters_per_pixel = MAP_SIZE_METERS / float(MAP_SIZE_PIXELS)
        self.CurrCarPose = None

        self.drawer = MapDrawer(MAP_SIZE_PIXELS, MAP_SIZE_METERS, title="My SLAM Map",
                                show_trajectory=True, origin_lower_left=True)

        self.enable_map_regularization = True
        self.regularization_params = {
            'min_free_cluster_size': 1000,
            'min_obstacle_thickness': 2,
            'wall_straightening_kernel': 3,
            'unknown_gap_threshold': 400,
            'wall_dilation_size': 3,
        }

    def UpdateMap(self, mapbytes, origin_lower_left=True):
        """核心方法：基于历史一致性投票的地图更新"""
        # 1. 获取当前SLAM输出
        slam_map = np.reshape(np.frombuffer(mapbytes, dtype=np.uint8),
                              (self.map_size_pixels, self.map_size_pixels))
        step1_start_time = time.time()
        #slam_map = self._regularize_slam_map(slam_map)
        elapsed = time.time()-step1_start_time
        #print(f"regularize耗时{elapsed}")

        # 2. 更新投票计数
        step2_start_time = time.time()
        self._update_votes(slam_map)
        elapsed = time.time()-step2_start_time
        #print(f"update_vote耗时{elapsed}")
        # 3. 基于投票结果更新锁定地图
        step3_start_time = time.time()
        self._update_locked_map()
        elapsed = time.time() - step3_start_time
        #print(f"update_locked map耗时{elapsed}")

        # 4. 生成最终显示地图：融合锁定信息和当前SLAM输出
        step4_start_time = time.time()
        final_map = self._generate_final_map(slam_map)
        elapsed = time.time() - step4_start_time
        #print(f"generate_final_map耗时{elapsed}")

        """
        # 5. 应用规整化（可选）
        if self.enable_map_regularization:
            final_map = self._regularize_slam_map(final_map)
        """

        # 6. 转换为显示格式
        self.grid = final_map.tolist()

    def _update_votes(self, slam_map: np.ndarray):
        """更新投票计数：简单粗暴，每个像素根据SLAM输出投票"""
        h, w = slam_map.shape

        # 对每个像素进行投票
        obstacle_mask = (slam_map <= OBSTACLE_THRESH)
        free_mask = (slam_map >= FREE_THRESH)

        # 只对非锁定区域进行投票（锁定区域不再变化）
        votable_mask = ~self.is_locked

        # 障碍物投票
        obstacle_vote_mask = obstacle_mask & votable_mask
        self.obstacle_votes[obstacle_vote_mask] += 1

        # 自由空间投票
        free_vote_mask = free_mask & votable_mask
        self.free_votes[free_vote_mask] += 1

    def _update_locked_map(self):
        """基于投票结果更新锁定地图"""
        obs_threshold = self.vote_params['obstacle_lock_threshold']
        free_threshold = self.vote_params['free_lock_threshold']
        conflict_ratio = self.vote_params['conflict_resolve_ratio']
        noise_threshold = self.vote_params['noise_filter_threshold']

        h, w = self.obstacle_votes.shape

        for y in range(h):
            for x in range(w):
                # 跳过已锁定的像素
                if self.is_locked[y, x]:
                    continue

                obs_votes = self.obstacle_votes[y, x]
                free_votes = self.free_votes[y, x]

                # 噪声过滤：投票数太少的不处理
                total_votes = obs_votes + free_votes
                if total_votes < noise_threshold:
                    continue

                # 决策逻辑
                lock_value = None

                # 情况1：障碍物票数足够且明显占优
                if (obs_votes >= obs_threshold ):
                    lock_value = 0  # 锁定为障碍物

                # 情况2：自由空间票数足够且明显占优
                elif (free_votes >= free_threshold ):
                    lock_value = 255  # 锁定为自由空间

                # 情况3：双方票数都很高但接近，选择票数更多的一方
                elif (obs_votes >= obs_threshold and free_votes >= free_threshold):
                    if obs_votes > free_votes * 1.2:
                        lock_value = 0
                    elif free_votes > obs_votes * 1.2:
                        lock_value = 255
                    # 否则暂时不锁定，继续观察

                # 执行锁定
                if lock_value is not None:
                    self.locked_map[y, x] = lock_value
                    self.is_locked[y, x] = True

    def _generate_final_map(self, slam_map: np.ndarray) -> np.ndarray:
        """生成最终地图：锁定区域使用历史值，非锁定区域使用SLAM输出"""
        final_map = slam_map.copy()

        # 锁定区域强制使用历史值
        locked_mask = self.is_locked
        final_map[locked_mask] = self.locked_map[locked_mask]

        return final_map

    def _regularize_slam_map(self, slam_map: np.ndarray) -> np.ndarray:
        """
        地图规整：现在需要保护锁定的区域不被规整算法破坏
        """
        #print("开始地图规整...")
        regularized = slam_map.copy()

        """
        # 步骤1：清理孤立的小自由区域，但保护锁定区域
        regularized = self._clean_isolated_free_areas_preserve_locked(regularized)
        print("完成孤立自由区域清理")

        # 步骤2：规整墙壁，但不改变锁定的障碍物
        regularized = self._straighten_walls_preserve_locked(regularized)
        print("完成墙壁规整")
        """
        # 步骤3：清理未知区域，考虑锁定信息
        regularized = self._clean_unknown_gaps_preserve_locked(regularized)
        #print("完成未知区域清理")

        return regularized

    def _clean_isolated_free_areas_preserve_locked(self, slam_map: np.ndarray) -> np.ndarray:
        """清理孤立自由区域，但保护锁定区域"""
        cleaned = slam_map.copy()

        # 只对非锁定区域进行孤立区域清理
        modifiable_mask = ~self.is_locked
        free_mask = (slam_map >= FREE_THRESH) & modifiable_mask

        if np.any(free_mask):
            labeled, num_features = ndimage.label(free_mask, structure=np.ones((3, 3)))

            for label_id in range(1, num_features + 1):
                component_mask = (labeled == label_id)
                component_size = np.sum(component_mask)

                if component_size < self.regularization_params['min_free_cluster_size']:
                    cleaned[component_mask] = 128

        return cleaned

    def _straighten_walls_preserve_locked(self, slam_map: np.ndarray) -> np.ndarray:
        """规整墙壁，但保护锁定的障碍物"""
        cleaned = slam_map.copy()

        # 识别可修改的障碍物（非锁定的障碍物）
        modifiable_obstacles = (slam_map <= OBSTACLE_THRESH) & (~self.is_locked)

        if np.any(modifiable_obstacles):
            # 对可修改障碍物应用规整
            kernel_size = self.regularization_params['wall_dilation_size']
            kernel = np.ones((kernel_size * 2 + 1, kernel_size * 2 + 1))
            dilated = ndimage.binary_dilation(modifiable_obstacles, kernel)

            h_walls, v_walls = self._extract_directional_walls(dilated)
            straightened_h = self._straighten_horizontal_walls(h_walls)
            straightened_v = self._straighten_vertical_walls(v_walls)
            combined_walls = straightened_h | straightened_v

            # 只在非锁定区域应用规整结果
            apply_mask = combined_walls & (~self.is_locked)
            cleaned[apply_mask] = 0

        # 确保锁定的障碍物不被改变
        locked_obstacles = self.is_locked & (self.locked_map <= OBSTACLE_THRESH)
        cleaned[locked_obstacles] = self.locked_map[locked_obstacles]

        return cleaned

    def _clean_unknown_gaps_preserve_locked(self, slam_map: np.ndarray) -> np.ndarray:
        """清理未知区域，但保护锁定区域"""
        cleaned = slam_map.copy()

        # 只处理非锁定的未知区域
        modifiable_unknown = ((slam_map > OBSTACLE_THRESH) &
                              (slam_map < FREE_THRESH) &
                              (~self.is_locked))

        if np.any(modifiable_unknown):
            labeled, num_features = ndimage.label(modifiable_unknown, structure=np.ones((3, 3)))

            for label_id in range(1, num_features + 1):
                component_mask = (labeled == label_id)
                component_size = np.sum(component_mask)

                if component_size <= self.regularization_params['unknown_gap_threshold']:
                    decision = self._analyze_unknown_gap_with_locks(slam_map, component_mask)

                    if decision == 'make_free':
                        cleaned[component_mask] = 255
                    elif decision == 'make_obstacle':
                        cleaned[component_mask] = 0

        return cleaned

    def _analyze_unknown_gap_with_locks(self, slam_map: np.ndarray, gap_mask: np.ndarray) -> str:
        """分析未知间隙，考虑锁定信息"""
        gap_positions = np.argwhere(gap_mask)

        free_neighbor_count = 0
        obstacle_neighbor_count = 0

        for y, x in gap_positions:
            for dy in [-1, 0, 1]:
                for dx in [-1, 0, 1]:
                    if dy == 0 and dx == 0:
                        continue
                    ny, nx = y + dy, x + dx
                    if 0 <= ny < slam_map.shape[0] and 0 <= nx < slam_map.shape[1]:

                        # 优先考虑锁定信息（权重更高）
                        if self.is_locked[ny, nx]:
                            locked_val = self.locked_map[ny, nx]
                            if locked_val >= FREE_THRESH:
                                free_neighbor_count += 3  # 锁定的自由空间权重更高
                            elif locked_val <= OBSTACLE_THRESH:
                                obstacle_neighbor_count += 6  # 锁定的障碍物权重更高
                        else:
                            # 非锁定区域使用当前SLAM值
                            val = slam_map[ny, nx]
                            if val >= FREE_THRESH:
                                free_neighbor_count += 1
                            elif val <= OBSTACLE_THRESH:
                                obstacle_neighbor_count += 3

        # 决策逻辑
        if free_neighbor_count > obstacle_neighbor_count * 1.5:
            return 'make_free'
        elif obstacle_neighbor_count > free_neighbor_count * 1.5:
            return 'make_obstacle'
        else:
            return 'keep_unknown'

    # === 保留原有规整化辅助方法 ===
    def _extract_directional_walls(self, obstacle_mask: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """提取水平和垂直方向的墙壁"""
        h_kernel = np.array([[0, 0, 0], [1, 1, 1], [0, 0, 0]])
        v_kernel = np.array([[0, 1, 0], [0, 1, 0], [0, 1, 0]])

        h_walls = ndimage.binary_opening(obstacle_mask, h_kernel, iterations=2)
        v_walls = ndimage.binary_opening(obstacle_mask, v_kernel, iterations=2)

        h_walls = ndimage.binary_dilation(h_walls, np.ones((3, 5)))
        v_walls = ndimage.binary_dilation(v_walls, np.ones((5, 3)))

        return h_walls, v_walls

    def _straighten_horizontal_walls(self, h_walls: np.ndarray) -> np.ndarray:
        """规整水平墙壁"""
        straightened = np.zeros_like(h_walls, dtype=bool)
        H, W = h_walls.shape

        for y in range(H):
            row = h_walls[y, :]
            segments = self._find_continuous_segments(row)
            for start_x, end_x in segments:
                if end_x - start_x >= 2:
                    straightened[y, start_x:end_x + 1] = True
                    for dy in [-1, 1]:
                        ny = y + dy
                        if 0 <= ny < H and np.any(h_walls[ny, start_x:end_x + 1]):
                            straightened[ny, start_x:end_x + 1] = True
        return straightened

    def _straighten_vertical_walls(self, v_walls: np.ndarray) -> np.ndarray:
        """规整垂直墙壁"""
        straightened = np.zeros_like(v_walls, dtype=bool)
        H, W = v_walls.shape

        for x in range(W):
            col = v_walls[:, x]
            segments = self._find_continuous_segments(col)
            for start_y, end_y in segments:
                if end_y - start_y >= 2:
                    straightened[start_y:end_y + 1, x] = True
                    for dx in [-1, 1]:
                        nx = x + dx
                        if 0 <= nx < W and np.any(v_walls[start_y:end_y + 1, nx]):
                            straightened[start_y:end_y + 1, nx] = True
        return straightened

    def _find_continuous_segments(self, line: np.ndarray) -> List[Tuple[int, int]]:
        """在一维数组中找到所有连续的True段"""
        segments = []
        start = None
        for i, val in enumerate(line):
            if val and start is None:
                start = i
            elif not val and start is not None:
                segments.append((start, i - 1))
                start = None
        if start is not None:
            segments.append((start, len(line) - 1))
        return segments

    # === 调试和监控方法 ===
    def get_vote_stats(self) -> dict:
        """获取投票统计信息，用于调试"""
        total_pixels = self.w * self.h
        locked_pixels = np.sum(self.is_locked)
        locked_obstacles = np.sum(self.is_locked & (self.locked_map <= OBSTACLE_THRESH))
        locked_free = np.sum(self.is_locked & (self.locked_map >= FREE_THRESH))

        return {
            'total_pixels': total_pixels,
            'locked_pixels': int(locked_pixels),
            'locked_obstacles': int(locked_obstacles),
            'locked_free': int(locked_free),
            'lock_ratio': float(locked_pixels / total_pixels),
            'max_obstacle_votes': int(np.max(self.obstacle_votes)),
            'max_free_votes': int(np.max(self.free_votes)),
            'avg_obstacle_votes': float(np.mean(self.obstacle_votes[self.obstacle_votes > 0])) if np.any(
                self.obstacle_votes) else 0,
            'avg_free_votes': float(np.mean(self.free_votes[self.free_votes > 0])) if np.any(self.free_votes) else 0,
        }

    def reset_votes_if_overlock(self):
        """防护机制：如果锁定过度，重置部分投票"""
        stats = self.get_vote_stats()
        if stats['lock_ratio'] > 0.7:  # 如果超过70%被锁定，可能有问题
            print(f"警告：锁定比例过高 ({stats['lock_ratio']:.1%})，重置低票数区域")

            # 重置票数较少的锁定区域
            low_confidence_mask = ((self.obstacle_votes + self.free_votes) < 6) & self.is_locked

            self.is_locked[low_confidence_mask] = False
            self.locked_map[low_confidence_mask] = 128
            self.obstacle_votes[low_confidence_mask] = 0
            self.free_votes[low_confidence_mask] = 0

    # === 保留原有接口 ===
    def m2pix(self, x_m, y_m):
        s = self.map_scale_meters_per_pixel
        return x_m / s, y_m / s

    def pix2m(self, x_pix, y_pix):
        s = self.map_scale_meters_per_pixel
        return x_pix * s, y_pix * s

    def SetCarPose(self, x_m, y_m, theta_deg):
        x_pix, y_pix = self.m2pix(x_m, y_m)
        self.CurrCarPose = (x_pix, y_pix, theta_deg)

    def GetCarPose(self):
        if self.CurrCarPose is None:
            raise RuntimeError("车辆位姿尚未设置")
        return self.CurrCarPose[0], self.CurrCarPose[1], self.CurrCarPose[2]

    def draw(self):
        # 在HUD中添加投票统计
        stats = self.get_vote_stats()
        extra_text = (f"Vote Stats:\n"
                      f"Locked: {stats['locked_pixels']} ({stats['lock_ratio']:.1%})\n"
                      f"Locked Obs: {stats['locked_obstacles']}\n"
                      f"Locked Free: {stats['locked_free']}")

        self.drawer.display(self.grid, self.CurrCarPose, extra_text=extra_text)
        print("drawing the map...")