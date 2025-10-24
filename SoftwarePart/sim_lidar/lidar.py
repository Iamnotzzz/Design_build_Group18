# sim_lidar.py
from typing import List, Tuple, Optional
import math
import random

from PythonRobotics.Localization.particle_filter.particle_filter import MAX_RANGE


class SimURG04LX:
    """
    URG-04LX 仿真雷达，输出与 BreezySLAM 期望一致：
    - 扫描线数: 682
    - 视场角:   240 度
    - 顺序:     以车头为 0°，数组索引从 -120° 开始，按逆时针增加到 +120°
                即 scan[0]→-120°，scan[341]→0°(正前)，scan[681]→+120°
    - 单位：    毫米（int）
    - 未命中：  返回 4000（或接近最大量程时根据 detection_margin 置 4000）

    依赖一个“地图容器”对象 map_obj，最少要有：
        map_obj.grid_map: 二维 0/1 列表 (grid[y][x])，1=障碍，0=通路
        map_obj.MAX_SIZE_PIXELS: int
        map_obj.MAX_SIZE_METERS: float
        map_obj.meter_per_pixel: float
    以及你维护的小车位姿 (x_m, y_m, theta_deg)。

    如需 LiDAR 安装偏置（相对小车参考点沿车头方向的平移），用 offset_mm 指定。
    """

    # 与 breezyslam.sensors.URG04LX 一致的规格
    SCAN_SIZE = 360
    FOV_DEG   = 360
    MAX_RANGE_MM = 20000
    SCAN_RATE_HZ = 10  # 仅作参考，不影响本函数输出

    def __init__(self,
                 map_obj,
                 detection_margin_mm: int = 0,
                 offset_mm: int = 0,
                 noise_sigma_mm: float = 0.0,
                 no_return_prob: float = 0.0,
                 step_fraction: float = 0.5):
        """
        detection_margin_mm: 接近最大量程的读数将被置为 MAX_RANGE_MM
        offset_mm:           LiDAR 相对小车参考点沿车头方向的前向偏置（毫米）
        noise_sigma_mm:      高斯测距噪声标准差（毫米），默认 0（无噪声）
        no_return_prob:      无回波概率（将该束置为 MAX_RANGE_MM）
        step_fraction:       射线步进长度 = meter_per_pixel * step_fraction
        """
        self.map = map_obj
        self.det_margin = max(0, int(detection_margin_mm))
        self.offset_m = float(offset_mm) / 1000.0
        self.noise_sigma = float(noise_sigma_mm)
        self.no_return_prob = float(no_return_prob)
        self.step_m = max(1e-6, self.map.meter_per_pixel * float(step_fraction))

        # 预计算角度步长
        self._dtheta_deg = self.FOV_DEG / (self.SCAN_SIZE - 1)
        self._half_fov_deg = self.FOV_DEG / 2.0

    # ------------ 公共接口 ------------

    def scan_from_pose(self, x_m: float, y_m: float, theta_deg: float) -> List[int]:
        """
        生成一帧扫描（毫米整型列表，长度 682）。
        theta_deg: 车体朝向（0°=x正方向，逆时针为正）。
        """
        # 应用 LiDAR 前向偏置（沿车头方向）
        theta_rad = math.radians(theta_deg)
        ox = x_m + self.offset_m * math.cos(theta_rad)
        oy = y_m + self.offset_m * math.sin(theta_rad)

        scans: List[int] = [self.MAX_RANGE_MM] * self.SCAN_SIZE

        for i in range(self.SCAN_SIZE):
            # 以车头为 0°，索引 0 对应 -120°，索引递增逆时针
            rel_deg = -self._half_fov_deg + i * self._dtheta_deg
            beam_rad = math.radians(theta_deg + rel_deg)

            # 随机判定无回波
            if self.no_return_prob > 0.0 and random.random() < self.no_return_prob:
                scans[i] = self.MAX_RANGE_MM
                continue

            # 射线步进
            dist_m = self._raycast_metric(ox, oy, beam_rad, self.MAX_RANGE_MM / 1000.0)
            dist_mm = int(round(dist_m * 1000.0))

            # 测距噪声
            if self.noise_sigma > 0.0 and dist_mm < self.MAX_RANGE_MM:
                dist_mm = int(round(self._gauss_clamp(dist_mm, self.noise_sigma, 0, self.MAX_RANGE_MM)))

            # detection_margin：接近最大量程的读数直接视为未命中
            if dist_mm >= self.MAX_RANGE_MM - self.det_margin:
                scans[i] = self.MAX_RANGE_MM
            else:
                scans[i] = max(0, min(self.MAX_RANGE_MM, dist_mm))

        return scans

    def scan(self) -> List[int]:
        """从 map_obj 当前车位姿获取扫描。"""
        x_m, y_m, theta_deg = self._require_pose()
        return self.scan_from_pose(x_m, y_m, theta_deg)

    # ------------ 内部工具 ------------

    def _require_pose(self) -> Tuple[float, float, float]:
        if not hasattr(self.map, "curr_car_pos") or self.map.curr_car_pos is None:
            raise RuntimeError("仿真地图未设置当前小车位姿，请先调用 set_car_pos(x_m, y_m, theta_deg)")
        return self.map.curr_car_pos

    def _raycast_metric(self, x_m: float, y_m: float, theta_rad: float, max_range_m: float) -> float:
        """
        简单的 DDA 栅格步进：从 (x_m,y_m) 沿 theta 前进，遇到障碍（grid=1）返回距离，
        越界即视为障碍。未命中返回 max_range_m。
        """
        step = self.step_m
        #print(f"step{step}")
        cos_t = math.cos(theta_rad)
        sin_t = math.sin(theta_rad)
        dist = 0.0

        mpp = self.map.meter_per_pixel
        grid = self.map.grid_map
        H = len(grid)
        W = len(grid[0]) if H > 0 else 0

        while dist < max_range_m:
            xm = x_m + dist * cos_t
            ym = y_m + dist * sin_t
            x_pix = int(round(xm / mpp))
            y_pix = int(round(ym / mpp))
            # 越界视为命中
            if x_pix < 0 or y_pix < 0 or x_pix >= W or y_pix >= H:
                return self.MAX_RANGE_MM
            # 命中障碍
            if grid[y_pix][x_pix] == 1:
                return dist
            dist += step

        return max_range_m

    @staticmethod
    def _gauss_clamp(mu: float, sigma: float, lo: float, hi: float) -> float:
        v = random.gauss(mu, sigma)
        return min(hi, max(lo, v))
