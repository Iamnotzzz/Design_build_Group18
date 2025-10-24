from sim.sim_robot.tool import Rover
import math

class simRover:
    def __init__(self,init_time_stamp_us):
        self.cur_time_stamp_us = init_time_stamp_us

        #左右轮编码器累计计数
        self._cum_left_ticks = 0
        self._cum_right_ticks = 0
        self.rov = Rover()

    def _wrap_deg(self,theta_deg: float) -> float:
        # 角度归一化到 [-180, 180)
        return (theta_deg + 180.0) % 360.0 - 180.0

    def computeposechange(self):

        dxy_mm, dtheta_deg, dt_s = self.rov.computePoseChange(
            (self.cur_time_stamp_us, self._cum_left_ticks, self._cum_right_ticks)
        )
        return dxy_mm, dtheta_deg, dt_s

    def get_ticks_time(self):
        return self._cum_left_ticks,self._cum_right_ticks,self.cur_time_stamp_us

    def set_ticks_time(self,cum_left_ticks,cum_right_ticks,time_stamp_us):
        self._cum_left_ticks = cum_left_ticks
        self._cum_right_ticks = cum_right_ticks
        self.cur_time_stamp_us = time_stamp_us

    def ComputeSimMapPose(self,prev_x_m,prev_y_m,prev_theta_deg,dxy_mm, dtheta_deg, dt_s):
        """
        通过角度变化和位移量得到新的仿真地图的位置
        """

        dxy_m = dxy_mm / 1000.0

        # 2) 先按“旧朝向”平移
        th_rad_prev = math.radians(prev_theta_deg)
        x_m = prev_x_m + dxy_m * math.cos(th_rad_prev)
        y_m = prev_y_m + dxy_m * math.sin(th_rad_prev)

        # 3) 再叠加转角
        theta_deg = self._wrap_deg(prev_theta_deg + dtheta_deg)

        return x_m, y_m, theta_deg, dt_s




