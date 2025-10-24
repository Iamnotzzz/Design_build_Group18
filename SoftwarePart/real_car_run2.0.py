import threading
import time

grid_MAP_SIZE_PIXELS = 300
grid_MAP_SIZE_METERS = 6

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
from sim.SerialIO.btlink import BTLink, WheelGeom as BTWheels, to_slam_inputs, ticks_to_motion
import argparse
import logging

p = Pose(0,0,0)

class threadfunc(threading.Thread):
    def __init__(self,link:BTLink,bt_geom:BTWheels,slam:RMHC_SLAM,lgm,planner:Planner,pose_vec,mapbytes,max_range_mm,report_box:dict):
        super().__init__(daemon=True)
        self.link = link
        self.bt_geom = bt_geom
        self.slam = slam
        self.lgm = lgm
        self.planner = planner
        self.pose = pose_vec
        self.mapbytes = mapbytes
        self.max_range = max_range_mm
        self._stop_evt = threading.Event()


        # 控制状态
        self.in_flight = False  # 是否已有命令在执行
        self.prev_state = None  # (cumL, cumR, ts_us)
        self.last_cmd_id = 0
        self.report = report_box

    def stop(self):
        self._stop_evt.set()

    def run(self):
        while not self._stop_evt.is_set():
            frm = self.link.get_latest_frame()
            if not frm:
                continue

            #stm->pc , slam input
            scan360, velocities, self.prev_state = to_slam_inputs(self.prev_state,frm,self.bt_geom,max_range_mm=self.max_range,quality_min=0)

            #更新slam地图
            new_ve = (0,0,0)
            self.slam.update(scan360,new_ve)
            self.pose[0], self.pose[1], self.pose[2] = self.slam.getpos()  # mm,mm,deg
            self.slam.getmap(self.mapbytes)
            self.lgm.UpdateMap(self.mapbytes)
            self.lgm.SetCarPose(self.pose[0] / 1000., self.pose[1] / 1000., self.pose[2])

            p.x_m = self.pose[0] / 1000.0
            p.y_m = self.pose[1] / 1000.0
            p.theta_deg = self.pose[2]


            while True:
                frm = self.link.get_latest_frame()
                if not frm:
                    continue
                #print(f"[UpFrame] ts={frm.time_us}  cmd_id={frm.cmd_id}  status={frm.status}")
                # 控制：仅在status==0时考虑下发下一段命令
                if frm.status == 0:
                    if self.in_flight:
                        self.in_flight = False
                        print("上一指令已完成")

                    # 没有在途指令：向planner索要下一段指令
                    if not self.in_flight:
                        command = self.planner.plan_next(p)
                        if command is None:
                            print("任务已完成，停止控制线程。")
                            break

                        turn_deg, straight_dist_m, self.report["report"] = command
                        turn_deg = 30
                        straight_dist_m = 0.
                        self.last_cmd_id += 1
                        self.link.send_cmd(self.last_cmd_id, turn_rad=turn_deg, distance_m=straight_dist_m)
                        self.in_flight = True
                        #print(f"cmd_id{self.last_cmd_id},turn_deg{turn_deg},straight_dist_m{straight_dist_m}")
                        sleep(0.4)

                        frm = self.link.get_latest_frame()
                        print(f"[UpFrame] ts={frm.time_us}  cmd_id={frm.cmd_id}  status={frm.status}")

                        # stm->pc , slam input
                        scan360, velocities, self.prev_state = to_slam_inputs(self.prev_state, frm, self.bt_geom,
                                                                              max_range_mm=self.max_range, quality_min=0)
                        new_ve = (straight_dist_m*1000,turn_deg,velocities[2])
                        # 更新slam地图
                        print(f"new_ve{new_ve}")
                        self.slam.update(scan360, new_ve)
                        self.pose[0], self.pose[1], self.pose[2] = self.slam.getpos()  # mm,mm,deg
                        self.slam.getmap(self.mapbytes)
                        self.lgm.UpdateMap(self.mapbytes)
                        self.lgm.SetCarPose(self.pose[0] / 1000., self.pose[1] / 1000., self.pose[2])

                        p.x_m = self.pose[0] / 1000.0
                        p.y_m = self.pose[1] / 1000.0
                        p.theta_deg = self.pose[2]
                        break

parser = argparse.ArgumentParser(description="PC↔STM32 蓝牙联测（只用 btlink.py）")
parser.add_argument("--port", type=str, default="COM7", help="串口名，如 COM7 或 /dev/rfcomm0；留空自动探测")
parser.add_argument("--baud", type=int, default=921600, help="波特率，默认 115200")
parser.add_argument("--wheel-radius", type=float, default=0.035, help="轮半径(米)")
parser.add_argument("--half-wheelbase", type=float, default=0.1, help="半轮距(米)")
parser.add_argument("--cpr", type=int, default=12656, help="编码器CPR")
parser.add_argument("--latest", action="store_true", help="只取最新帧（丢旧保新）")
parser.add_argument("--csv", type=str, default=None, help="将关键字段记录到此CSV文件")
parser.add_argument("--log", type=str, default="INFO", help="日志等级：DEBUG/INFO/WARNING/ERROR")
args = parser.parse_args()



def main():
    geom = BTWheels(args.wheel_radius, args.half_wheelbase, args.cpr)
    link = BTLink(port=args.port, baud=args.baud, logger=logging.getLogger("BTLink"))
    link.start()


    # Allocate byte array to receive map updates
    mapbytes = bytearray(grid_MAP_SIZE_PIXELS * grid_MAP_SIZE_PIXELS)

    #初始化仿真小车
    sim_robot = simRover(0)

    laser = Laser(360,5.0,360.0,8000,0,0)
    #初始化slam
    slam = Deterministic_SLAM(laser, grid_MAP_SIZE_PIXELS, grid_MAP_SIZE_METERS,hole_width_mm=90)

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

    ctrl = threadfunc(link, geom, slam, lgm, planner, pose, mapbytes,max_range_mm=8000,report_box=report_box)
    ctrl.start()

    # --- 主循环：SLAM 图
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
        #print(f"渲染lgm耗时{elapsed}")


if __name__ == "__main__":
    main()