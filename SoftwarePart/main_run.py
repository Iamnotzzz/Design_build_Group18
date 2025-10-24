import time

grid_MAP_SIZE_PIXELS = 800
grid_MAP_SIZE_METERS = 4

sim_MAP_SIZE_METERS = 2

SEED = 9999
from breezyslam.algorithms import Deterministic_SLAM, RMHC_SLAM
from sim.sim_robot.tool import MinesLaser
from breezyslam.sensors import Laser
from sim.world.sim_maze import sim_map
from sim.world.maze_loader import load_map,scale_grid,scale_grid_widen_passages
from sim.sim_lidar.lidar import SimURG04LX
from sim.sim_robot.robot import simRover
from sim.ListMap.grid_map import ListGridMap
from threading import Thread
from sim.plan.planning import Planner, WheelGeom, VelocityLimits, CmdTolerance,Pose, PlannerStateReport,MissionPhase,EncoderTarget
from time import sleep

def threadfunc(robot:simRover, slam:RMHC_SLAM, timestamps, mapbytes,
               pose,lidar:SimURG04LX,planner:Planner,report_box:dict,
               sim_maze:sim_map,lgm:ListGridMap):
    # Initialize time for delay
    prevtime = 0
    p = Pose(x_m=0.0,y_m=0.0,theta_deg = 0)
    time_us = timestamps

    scan = lidar.scan()
    #print(f"scanning:{scan}")
    velocities = robot.computeposechange()

    slam.update(scan, velocities)
    pose[0], pose[1], pose[2] = slam.getpos()  # 注意！这里获取的是坐标单位是mm
    slam.getmap(mapbytes)
    lgm.UpdateMap(mapbytes)

    print(f"pose{pose[0],pose[1],pose[2]}")
    lgm.SetCarPose(pose[0] / 1000., pose[1] / 1000., pose[2])


    cur_cum_left_ticks, cur_cum_right_ticks, cur_time_stamp = robot.get_ticks_time()
    #print(f"cur_cum_left_ticks{cur_cum_left_ticks},cur_cum_right_ticks{cur_cum_right_ticks},cur_time_stamp{cur_time_stamp}")

    p.x_m = pose[0] / 1000.0
    p.y_m = pose[1] / 1000.0
    p.theta_deg = pose[2]
    command, rep = planner.plan_next(p, (cur_cum_left_ticks, cur_cum_right_ticks))
    print(f"command{command}")
    report_box["report"] = rep

    while True:

        #小车执行命令
        for encoder_target in command:
            start_time = time.time()

            time_us = time_us + (encoder_target.duration_ms*1000)
            robot.set_ticks_time(encoder_target.cum_left_ticks,encoder_target.cum_right_ticks,time_us)

            dxy_mm, dtheta_deg, dt_s = robot.computeposechange()

            #处理仿真地图上的小车位姿变化
            sim_car_pose = sim_maze.get_car_pos()  # sim_car_pose是个元组
            x_m, y_m, theta_deg, dt_s = robot.ComputeSimMapPose(sim_car_pose[0],sim_car_pose[1],sim_car_pose[2],dxy_mm, dtheta_deg, dt_s)
            sim_maze.set_car_pos(x_m,y_m,theta_deg)
            #估计slam栅格地图上的小车位子变化
            scan = lidar.scan()

            print(f"scanning:{scan}  len:{len(scan)}")
            slam.update(scan, (dxy_mm, dtheta_deg, dt_s))
            pose[0], pose[1], pose[2] = slam.getpos()  # 注意！这里获取的是坐标单位是mm
            slam.getmap(mapbytes)
            lgm.UpdateMap(mapbytes)

            lgm.SetCarPose(pose[0] / 1000., pose[1] / 1000., pose[2])
            """
            currtime = time_us / 1.e6  # Convert usec to sec
            if prevtime > 0:
                sleep(currtime - prevtime)
            prevtime = currtime
            """
            dur_time = time.time()-start_time
            print(f"durtime{dur_time}")



        robot.set_ticks_time(command[-1].cum_left_ticks,command[-1].cum_right_ticks,time_us)
        cur_cum_left_ticks, cur_cum_right_ticks, cur_time_stamp = robot.get_ticks_time()
        p.x_m = pose[0] / 1000.0
        p.y_m = pose[1] / 1000.0
        p.theta_deg = pose[2]

        command, rep = planner.plan_next(p, (cur_cum_left_ticks, cur_cum_right_ticks))

        report_box["report"] = rep

        if rep.phase == MissionPhase.DONE:
            break



def main():
    ori_grid, ori_start_pix = load_map("2.json")
    grid ,start_pix= scale_grid_widen_passages(ori_grid,ori_start_pix,200,2)

    sim_maze = sim_map(grid_binary=grid, MAX_SIZE_METERS=sim_MAP_SIZE_METERS, start_point=start_pix)
    start_m = sim_maze.pix2m(start_pix[0]+1,start_pix[1]+4)
    sim_maze.set_car_pos(start_m[0],start_m[1],90) #默认迷宫的入口在底边，

    # 2) 创建仿真雷达
    lidar = SimURG04LX(sim_maze, detection_margin_mm=0, offset_mm=0, noise_sigma_mm=0.0)

    # Allocate byte array to receive map updates
    mapbytes = bytearray(grid_MAP_SIZE_PIXELS * grid_MAP_SIZE_PIXELS)

    #初始化仿真小车
    sim_robot = simRover(0)

    laser = Laser(360,10.0,360.0,8000,0,0)
    #初始化slam
    slam = Deterministic_SLAM(laser, grid_MAP_SIZE_PIXELS, grid_MAP_SIZE_METERS,hole_width_mm=70)

    #创建slam栅格图
    lgm = ListGridMap(grid_MAP_SIZE_PIXELS, grid_MAP_SIZE_METERS)

    # Pose will be modified in our threaded code
    pose = [0, 0, 0]

    #初始化路径规划模块
    wheel_geom = WheelGeom(
        wheel_radius_mm=77,
        half_axle_len_mm=165,
        ticks_per_cycle=2000
    )
    planner = Planner(lgm,wheel_geom,vel_limits=VelocityLimits(),tol=CmdTolerance())

    report = PlannerStateReport(
        phase=MissionPhase.EXPLORE,
        curr_pose=Pose(0.0, 0.0, 0.0),
        next_waypoint=None,
        frontier_count=0,
        known_maze_rect=None,
        has_found_exit=False,
        path_len_nodes=0,
        note="未初始化"
    )

    report_box = {"report": report}

    thread = Thread(target=threadfunc,args=(sim_robot,slam,0,mapbytes,pose,lidar,planner,report_box,sim_maze,lgm))
    thread.daemon = True
    thread.start()

    # ===== ③ 在 main() 末尾添加“双窗体可视化循环” =====
    # --- 构造仿真地图的灰度图（一次性）: 1->0(墙), 0->255(通路)
    sim_grid = [[0 if v == 1 else 255 for v in row] for row in sim_maze.grid_map]

    # --- 新建一个 Drawer 用于仿真地图（右窗）
    from ListMap.draw_map import MapDrawer

    sim_drawer = MapDrawer(map_size_pixels=len(sim_grid[0]),
                           map_size_meters=sim_maze.MAX_SIZE_METERS,
                           title="Sim Map (Ground Truth)",
                           show_trajectory=True, origin_lower_left=True)

    # --- 主循环：同时刷新 SLAM 与 Sim 两张图
    import time as _t
    while True:
        # 读取最新 Planner 报告（若线程还未写入，用初值兜底）
        rep = report_box.get("report", None)

        # 1) SLAM 栅格与位姿（lgm 已被线程实时更新）
        slam_pose = lgm.CurrCarPose  # (x_pix, y_pix, theta_deg) or None
        hud = ""
        if rep is not None:
            hud = (
                f"Phase: {rep.phase.name}\n"
                f"Frontiers: {rep.frontier_count}\n"
                f"Known ROI: {rep.known_maze_rect}\n"
                f"Path nodes: {rep.path_len_nodes}\n"
                f"Found exit: {rep.has_found_exit}\n"
                f"Note: {rep.note}"
            )
        # 传入前沿点信息
        frontier_points = rep.frontier_points if rep is not None else None
        best_frontier = rep.best_frontier_point if rep is not None else None
        spacialty = rep.Spatially_sampled
        start_time = time.time()
        lgm.drawer.display(lgm.grid, slam_pose, extra_text=hud,roi_rect=(rep.known_maze_rect if rep is not None else None),frontier_points=frontier_points,best_frontier_point=best_frontier,spatially_sampled=spacialty,exit_pose_pix=rep.exit_pose)  # <<< 传入 HUD
        elapsed = time.time()-start_time
        print(f"渲染lgm耗时{elapsed}")

        # 2) 仿真真值图与真值位姿
        sx, sy, sth = sim_maze.get_car_pos()
        spx, spy = sim_maze.m2pix(sx, sy)
        sim_drawer.display(sim_grid, (spx, spy, sth))


if __name__ == "__main__":
    main()