from typing import List,Tuple

class sim_map:
    def __init__(self,grid_binary: List[List[int]],MAX_SIZE_METERS,start_point):
        self.grid_map = grid_binary
        self.MAX_SIZE_PIXELS = len(grid_binary[0])
        self.MAX_SIZE_METERS = MAX_SIZE_METERS
        self.meter_per_pixel = MAX_SIZE_METERS/float(self.MAX_SIZE_PIXELS)
        self.start_point = start_point
        self.curr_car_pos = None

    def set_car_pos(self,x_m,y_m,theta_deg):
        self.curr_car_pos = (x_m,y_m,theta_deg)

    def get_car_pos(self):
        return self.curr_car_pos

    def m2pix(self,x_m,y_m):
        s = self.meter_per_pixel
        return x_m/s, y_m/s

    def pix2m(self,x_pix,y_pix):
        s = self.meter_per_pixel
        return x_pix*s, y_pix*s


