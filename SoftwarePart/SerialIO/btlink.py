# -*- coding: utf-8 -*-
"""
btlink.py — PC 上位机与 STM32（HC-04）蓝牙通信（最新版协议）

下行（PC→STM32）命令帧：
  0xAA 0x55 | uint16 len=10 | uint16 cmd_id | float32 turn_rad | float32 distance_m | uint16 CRC16

上行（STM32→PC）周期数据帧（每 200 ms）：
  0x55 0xAA |
    uint32 time_us |
    uint16 cmd_id |
    uint8  status |   # 0=停止(完成), 1=转弯, 2=直行
    int32  encoder_l |
    int32  encoder_r |
    float32 current_yaw |
    uint16 data_count |
    [ data_count * { uint8 Quality, uint16 angle_q6, uint16 distance_q2 } ]

角度解码：angle_deg = angle_q6 / 64.0
距离解码：distance_mm = distance_q2 / 4.0

提供：
- BTLink: 起停、收发、自动重连、取帧
- send_cmd(cmd_id, turn_rad, distance_m)
- get_frame()/get_latest_frame()
- to_slam_inputs(prev_state, frame, geom, max_range_mm, quality_min): 生成 (scan360_mm, (dxy_mm, dtheta_deg, dt_s))

依赖：pyserial
"""

from __future__ import annotations
import struct
import threading
import time
import logging
from dataclasses import dataclass
from typing import Optional, List, Tuple
from queue import Queue, Full, Empty

import serial
import serial.tools.list_ports


# ================= 常量与工具 =================

# 帧头常量（方向不同，头字节顺序不同）
HEADER_DOWN = b'\xAA\x55'  # PC → STM32
HEADER_UP   = b'\x55\xAA'  # STM32 → PC

DEFAULT_BAUD = 921600
DEFAULT_TIMEOUT = 0.1       # 串口 read 超时(s)
RECONNECT_INTERVAL = 2.0    # 自动重连间隔(s)

# CRC16(Modbus) — init 0xFFFF, poly 0xA001
def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if (crc & 1) != 0:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# ================= 数据结构 =================

@dataclass
class WheelGeom:
    wheel_radius_m: float      # 轮半径 R（米）
    half_wheelbase_m: float    # 半轮距 B（米）
    ticks_per_rev: int         # 编码器 CPR

@dataclass
class LidarPoint:
    quality: int
    angle_deg: float
    distance_mm: float

@dataclass
class UpFrame:  # STM32 → PC
    time_us: int
    cmd_id: int
    status: int        # 0=完成, 1=转弯, 2=直行
    encoder_l: int
    encoder_r: int
    current_yaw: float
    points: List[LidarPoint]   # data_count 个点（原始角距）

# ================ BTLink 主类 =================

class BTLink:
    """
    管理串口：起停、收发、自动重连、解析上行帧、发送命令帧。

    典型用法：
        link = BTLink('COM7', 115200)
        link.start()
        link.send_cmd(cmd_id=1, turn_rad=0.2, distance_m=0.15)
        frm = link.get_frame(timeout=1.0)     # 最旧帧
        latest = link.get_latest_frame()      # 最新帧
        link.stop()
    """

    def __init__(self,
                 port: Optional[str],
                 baud: int = DEFAULT_BAUD,
                 timeout: float = DEFAULT_TIMEOUT,
                 auto_reconnect: bool = True,
                 max_queue: int = 1,
                 logger: Optional[logging.Logger] = None):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.auto_reconnect = auto_reconnect
        self.log = logger or logging.getLogger("BTLink")
        self._ser: Optional[serial.Serial] = None
        self._stop_evt = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._rx_buf = bytearray()
        self._queue: Queue[UpFrame] = Queue(maxsize=max_queue)

    # ---------- 生命周期 ----------
    def start(self):
        self._stop_evt.clear()
        self._open_serial()
        self._rx_thread = threading.Thread(target=self._rx_loop, name="BTLink-RX", daemon=True)
        self._rx_thread.start()
        self.log.info("BTLink started.")

    def stop(self):
        self._stop_evt.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=1.5)
        self._close_serial()
        self.log.info("BTLink stopped.")

    # ---------- 下行发送（PC→STM32） ----------
    def send_cmd(self, cmd_id: int, turn_rad: float, distance_m: float):
        """
        发送命令帧：
          AA 55 | len=10 | cmd_id(u16) | turn_rad(f32) | distance_m(f32) | CRC16
        """
        payload = struct.pack('<Hff', int(cmd_id), float(turn_rad), float(distance_m))
        # len=10 只计算 payload 长度
        body_wo_crc = HEADER_DOWN + struct.pack('<H', 10) + payload
        crc = struct.pack('<H', crc16_modbus(body_wo_crc))
        frame = body_wo_crc + crc
        self._write(frame)
        self.log.debug(f"send_cmd: id={cmd_id} turn={turn_rad:.4f}rad dist={distance_m:.4f}m")

    # ---------- 上行取帧（STM32→PC） ----------
    def get_frame(self, timeout: Optional[float] = None) -> Optional[UpFrame]:
        """FIFO 取一帧（最旧帧）。"""
        try:
            return self._queue.get(timeout=timeout)
        except Empty:
            return None

    def get_latest_frame(self) -> Optional[UpFrame]:
        """只取最新一帧（丢弃队列里较旧的）。"""
        try:
            frm = self._queue.get(timeout=0.0)
        except Empty:
            return None
        # 清空剩余，保留最新
        while True:
            try:
                frm = self._queue.get_nowait()
            except Empty:
                break
        return frm

    # ---------- 私有：串口 ----------
    def _open_serial(self):
        while not self._stop_evt.is_set():
            try:
                if self._ser and self._ser.is_open:
                    return
                if self.port is None:
                    self.port = self._auto_pick_port()
                self._ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
                self.log.info(f"Serial opened: {self.port} @ {self.baud}")
                return
            except Exception as e:
                self.log.warning(f"Open serial failed: {e}")
                if not self.auto_reconnect:
                    raise
                time.sleep(RECONNECT_INTERVAL)

    def _close_serial(self):
        try:
            if self._ser:
                self._ser.close()
        except Exception:
            pass
        finally:
            self._ser = None

    def _write(self, data: bytes):
        if not self._ser or not self._ser.is_open:
            self._open_serial()
        try:
            self._ser.write(data)
            self._ser.flush()
        except Exception as e:
            self.log.error(f"Serial write failed: {e}")
            if self.auto_reconnect:
                self._close_serial()

    # ---------- RX 线程 ----------
    def _rx_loop(self):
        buf = self._rx_buf
        while not self._stop_evt.is_set():
            # 确保串口已打开
            if not self._ser or not self._ser.is_open:
                if not self.auto_reconnect:
                    time.sleep(0.1); continue
                self._open_serial()
                time.sleep(0.1); continue

            try:
                chunk = self._ser.read(512)
                if chunk:
                    buf += chunk
                    # 解析尽可能多的上行帧
                    while True:
                        frm = self._try_parse_upframe(buf)
                        if frm is None:
                            break
                        # 入队（满则丢旧保新）
                        try:
                            self._queue.put_nowait(frm)
                        except Full:
                            try:
                                print("丢弃最旧的一帧")
                                _ = self._queue.get_nowait()
                            except Empty:
                                pass
                            self._queue.put_nowait(frm)
                else:
                    # read 超时：允许空转
                    pass
            except Exception as e:
                self.log.warning(f"Serial read error: {e}")
                if self.auto_reconnect:
                    self._close_serial()
                    time.sleep(RECONNECT_INTERVAL)
                else:
                    break

    # ---------- 上行帧解析（无总长度/CRC，靠头与 data_count 推断） ----------
    def _try_parse_upframe(self, buf: bytearray) -> Optional[UpFrame]:
        """
        从 buf 开头解析一帧 STM32→PC 上行数据帧：
          头 0x55 0xAA
          固定 21 字节 + 5*data_count
        成功则从 buf 移除该帧，并返回 UpFrame；否则返回 None（等待更多字节）。
        """
        # 找帧头
        start = buf.find(HEADER_UP)
        if start == -1:
            # 没找到头，丢弃无用前缀
            if len(buf) > 2048:
                del buf[:len(buf)-2]
            return None
        # 将无用前缀去掉
        if start > 0:
            del buf[:start]

        # 需要至少 HEADER + 固定字段（不含点）：2 + 4+2+1+4+4+4+2 = 23
        if len(buf) < 23:
            return None

        # 解析固定字段
        # 偏移：2(头之后) 开始
        try:
            time_us, = struct.unpack_from('<I', buf, 2)
            cmd_id,  = struct.unpack_from('<H', buf, 6)
            status   = buf[8]
            encoder_l, = struct.unpack_from('<i', buf, 9)
            encoder_r, = struct.unpack_from('<i', buf, 13)
            current_yaw, = struct.unpack_from('<f', buf, 17)
            data_count,  = struct.unpack_from('<H', buf, 21)
        except struct.error:
            return None

        if data_count > 2048:  # 合理性保护
            # 异常：丢掉一个字节，继续找头
            del buf[0]
            return None

        total_len = 23 + data_count * 5  # 每点 1+2+2 = 5 字节
        if len(buf) < total_len:
            return None  # 数据未收齐

        # 解析点云
        points: List[LidarPoint] = []
        off = 23
        for _ in range(data_count):
            q = buf[off]
            angle_q6, = struct.unpack_from('<H', buf, off + 1)
            dist_q2,  = struct.unpack_from('<H', buf, off + 3)
            off += 5
            angle_q6 = angle_q6 & 0x7FFF
            angle_deg = angle_q6 / 64.0
            distance_mm = dist_q2 / 4.0
            points.append(LidarPoint(q, angle_deg, distance_mm))

        # 移除整帧
        del buf[:total_len]

        return UpFrame(
            time_us=time_us,
            cmd_id=cmd_id,
            status=status,
            encoder_l=encoder_l,
            encoder_r=encoder_r,
            current_yaw=current_yaw,
            points=points
        )

    @staticmethod
    def _auto_pick_port() -> Optional[str]:
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            name = (p.description or '') + ' ' + (p.device or '')
            if 'Bluetooth' in name or 'HC' in name or 'SPP' in name:
                return p.device
        return ports[0].device if ports else None


# ================= 辅助：上行帧 → SLAM 输入 =================

def ticks_to_motion(prev: Tuple[int, int, int],
                    curr: Tuple[int, int, int],
                    geom: WheelGeom) -> Tuple[float, float, float]:
    """
    由两帧 (cumL, cumR, ts_us) 计算 BreezySLAM 需要的 (dxy_mm, dtheta_deg, dt_s)
    """
    cumL0, cumR0, ts0 = prev
    cumL1, cumR1, ts1 = curr
    dL = cumL1 - cumL0
    dR = cumR1 - cumR0
    revL = dL / float(geom.ticks_per_rev)
    revR = dR / float(geom.ticks_per_rev)
    sL = (2.0 * 3.141592653589793 * geom.wheel_radius_m) * revL
    sR = (2.0 * 3.141592653589793 * geom.wheel_radius_m) * revR
    ds = 0.5 * (sL + sR)
    dtheta_rad = (sR - sL) / (2.0 * geom.half_wheelbase_m)
    dt_s = (ts1 - ts0)  / 1e6
    return (ds * 1000.0, dtheta_rad * 180.0 / 3.141592653589793, dt_s)


def build_scan360(points: List[LidarPoint],
                  max_range_mm: int = 8000,
                  quality_min: int = 0,
                  do_interpolate: bool = False,
                  do_median3: bool = False) -> List[int]:
    """
    将 {angle_deg, distance_mm} 点集映射为 0..359 的距离数组（单位 mm）
    - 对相同角度（四舍五入到最近整数）取更小距离
    - 缺失角度用“环形线性插值”填补（避免突兀出现 max_range_mm）
    - 可选 3 点中值滤波平滑
    """
    # 第一步：聚合最小距离，先用 None 表示缺失
    vals: List[Optional[int]] = [None] * 360
    for p in points:

        a = p.angle_deg % 360.0
        idx = int(round(a)) % 360
        d = int(max(0, min(65535, round(p.distance_mm))))
        if d <= 0:
            continue
        if vals[idx] is None or d < vals[idx]:
            vals[idx] = d

    # 如果全缺失，直接返回全 max_range_mm
    if all(v is None for v in vals):
        print("全缺失")
        return [max_range_mm] * 360


    # 把仍为 None 的位置填为邻近有效值或 max_range_mm（极少数边角）
    # 这里用最近邻回填（双向各找最多 180°）
    for idx in range(360):
        if vals[idx] is None:
            # 向左右各搜最近的有效值
            d = 1
            dl = dr = None
            while d <= 180 and (dl is None or dr is None):
                if dl is None:
                    v = vals[(idx - d) % 360]
                    if v is not None:
                        dl = v
                if dr is None:
                    v = vals[(idx + d) % 360]
                    if v is not None:
                        dr = v
                d += 1
            if dl is None and dr is None:
                vals[idx] = max_range_mm
            elif dl is None:
                vals[idx] = dr
            elif dr is None:
                vals[idx] = dl
            else:
                vals[idx] = int(round(0.5 * (dl + dr)))

    scan = [int(v) if v is not None else 10 for v in vals]



    return reorder_scan_ccw_minus180_to_plus179(scan)

def reorder_scan_ccw_minus180_to_plus179(scan: List[int]) -> List[int]:
    """
    输入：scan[0..359]，从前方开始顺时针（0°→359°）
    输出：out[0..359]，逆时针角度从 -180°→+179°
    """

    part1 = scan[:181]
    part2 = scan[181:360]
    part1.reverse()
    part2.reverse()
    #print(f"length of scan {len(part1+part2)}")
    return part1 + part2

def to_slam_inputs(prev_state: Optional[Tuple[int, int, int]],
                   frame: UpFrame,
                   geom: WheelGeom,
                   max_range_mm: int = 8000,
                   quality_min: int = 0
                   ) -> Tuple[List[int], Tuple[float, float, float], Tuple[int, int, int]]:
    """
    把一帧上行数据转换为：
      scan360_mm, (dxy_mm, dtheta_deg, dt_s), curr_state
    供 BreezySLAM.update(scan, velocities) 使用
    """
    scan360 = build_scan360(frame.points, max_range_mm=max_range_mm, quality_min=quality_min)
    curr_state = (frame.encoder_l, frame.encoder_r, frame.time_us)
    if prev_state is None:
        velocities = (0.0, 0.0, 1e-6)
    else:
        velocities = ticks_to_motion(prev_state, curr_state, geom)
    return scan360, velocities, curr_state
