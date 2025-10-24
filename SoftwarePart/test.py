# main_sim_scan.py
from world.sim_maze import sim_map
from sim_lidar.lidar import SimURG04LX

# 1) 构造你的二值地图（示例：16x16，四周墙）
N = 16
grid = [[0]*N for _ in range(N)]
for i in range(N):
    grid[0][i] = grid[N-1][i] = grid[i][0] = grid[i][N-1] = 1  # 边界墙

for i in range(N):
    print(grid[i])

m = sim_map(grid_binary=grid, MAX_SIZE_METERS=3.2, start_point=(1,1))
m.set_car_pos(0.8, 0.8, 180.0)  # x=0.8m, y=0.8m, 朝向=0°(向右)

# 2) 创建仿真雷达（参数对齐 URG-04LX）
lidar = SimURG04LX(m, detection_margin_mm=70, offset_mm=145, noise_sigma_mm=0.0)

# 3) 生成一帧扫描（毫米整型列表，长度 682）
scan_mm = lidar.scan()

print(len(scan_mm), scan_mm[0], scan_mm[len(scan_mm)//2], scan_mm[-1])
# 应看到：中间束（正前方）是最近的墙距（毫米），两端接近 4000
