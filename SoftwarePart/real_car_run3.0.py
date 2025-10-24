import threading
import time

grid_MAP_SIZE_PIXELS = 800
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

import sys
import queue
import math

class CommandInputThread(threading.Thread):
    """
    在控制台读取手动指令：<turn_deg> <distance_m>
    示例：
      90 0        # 左转90°，不直行
      0  0.5      # 直行0.5米
      -45 0.2     # 右转45°，直行0.2米
    输入 'q' 或 Ctrl+C 结束。
    """
    def __init__(self, cmd_queue: "queue.Queue[tuple[float,float]]"):
        super().__init__(daemon=True)
        self.cmd_queue = cmd_queue
        self._stop_evt = threading.Event()

    def stop(self):
        self._stop_evt.set()

    def run(self):
        print("手动模式：请输入 '<转角度> <直行米>'，如 '90 0' 或 '0 0.5'；输入 'q' 退出。")
        while not self._stop_evt.is_set():
            try:
                line = sys.stdin.readline()
                if not line:
                    time.sleep(0.05); continue
                line = line.strip()
                if line.lower() in ("q", "quit", "exit"):
                    print("输入线程退出。")
                    break
                if not line:
                    continue
                parts = line.replace(",", " ").split()
                if len(parts) != 2:
                    print("格式错误：请输两个数，如 '30 0.2'")
                    continue
                turn_deg = float(parts[0])
                dist_m   = float(parts[1])
                self.cmd_queue.put((turn_deg, dist_m))
                print(f"[排队] turn={turn_deg:.3f}°  dist={dist_m:.3f} m")
            except Exception as e:
                print(f"[输入线程] 异常：{e}")



class threadfunc(threading.Thread):
    def __init__(self,link:BTLink,bt_geom:BTWheels,slam:RMHC_SLAM,lgm,planner:Planner,pose_vec,mapbytes,max_range_mm,report_box:dict,cmd_queue: "queue.Queue[tuple[float,float]]"):
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
        self.cmd_queue = cmd_queue  # ← 新增：手动指令队列


        # 控制状态
        self.in_flight = False  # 是否已有命令在执行
        self.prev_state = None  # (cumL, cumR, ts_us)
        self.last_cmd_id = 0
        self.report = report_box

    def stop(self):
        self._stop_evt.set()

    def run(self):
        current_cmd = 0
        while not self._stop_evt.is_set():
            frm = self.link.get_frame(timeout=1.0)
            print("car"*50)
            print(f"cmd_id{frm.cmd_id},status{frm.status}")
            print("car" * 50)
            if not frm:
                continue

            #stm->pc , slam input
            scan360, velocities, self.prev_state = to_slam_inputs(self.prev_state,frm,self.bt_geom,max_range_mm=self.max_range,quality_min=0)

            #更新slam地图
            #print(velocities)
            self.slam.update(scan360,velocities)
            self.pose[0], self.pose[1], self.pose[2] = self.slam.getpos()  # mm,mm,deg
            self.slam.getmap(self.mapbytes)
            self.lgm.UpdateMap(self.mapbytes)
            self.lgm.SetCarPose(self.pose[0] / 1000., self.pose[1] / 1000., self.pose[2])

            p.x_m = self.pose[0] / 1000.0
            p.y_m = self.pose[1] / 1000.0
            p.theta_deg = self.pose[2]


            # 控制：仅在status==0时考虑下发下一段命令
            if frm.status == 0 and frm.cmd_id == self.last_cmd_id:
                if self.in_flight:
                    self.in_flight = False
                    print("上一指令已完成")

                #没有在途指令：向planner索要下一段指令
                if not self.in_flight:
                    # —— 关键：队列可能暂时没有指令，优雅处理 ——
                    try:
                        # 等待最多 0.1s 获取一条新指令；拿不到就跳过本帧
                        turn_deg, straight_dist_m = self.cmd_queue.get(timeout=0.1)
                    except queue.Empty:
                        # 没有新指令就先不发，等下一帧
                        continue



                    self.last_cmd_id +=1
                    current_cmd = self.last_cmd_id
                    self.link.send_cmd(self.last_cmd_id,turn_rad=turn_deg, distance_m=straight_dist_m)

                    self.in_flight = True
                    print("#"*50)
                    print(f"cmd_id{self.last_cmd_id},turn_deg{turn_deg},straight_dist_m{straight_dist_m}")
                    print("#" * 50)


parser = argparse.ArgumentParser(description="PC↔STM32 蓝牙联测（只用 btlink.py）")
parser.add_argument("--port", type=str, default="COM7", help="串口名，如 COM7 或 /dev/rfcomm0；留空自动探测")
parser.add_argument("--baud", type=int, default=921600, help="波特率，默认 115200")
parser.add_argument("--wheel-radius", type=float, default=0.034, help="轮半径(米)")
parser.add_argument("--half-wheelbase", type=float, default=0.0955, help="半轮距(米)")
parser.add_argument("--cpr", type=int, default=1394, help="编码器CPR")
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

    laser = Laser(360,5.0,360.0,500,0,0)
    #初始化slam
    slam = Deterministic_SLAM(laser, grid_MAP_SIZE_PIXELS, grid_MAP_SIZE_METERS,hole_width_mm=100)

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
    # ← 新增：命令队列 & 输入线程
    cmd_queue: "queue.Queue[tuple[float,float]]" = queue.Queue(maxsize=100)
    cmd_input = CommandInputThread(cmd_queue)
    cmd_input.start()

    ctrl = threadfunc(link, geom, slam, lgm, planner, pose, mapbytes,max_range_mm=8000,report_box=report_box,cmd_queue=cmd_queue)
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