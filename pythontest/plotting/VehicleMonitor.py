from pymavlink import mavutil
from threading import Thread, Lock
import math
import time

class VehicleMonitor:
    def __init__(self, name, udpPort, save_CSV = True, color='blue', enableDroneWindow=True, enableBoatWindow=True, enableGlobalWindow=True):
        self.SIM_running = False
        self.save_CSV = save_CSV
        self.name = name
        self.udpPort = udpPort
        self.color = color

        self.enableDroneWindow = enableDroneWindow
        self.enableBoatWindow = enableBoatWindow
        self.enableGlobalWindow = enableGlobalWindow

        self.posHistory = []     # List of (time, x, y, z, vx, vy, vz)
        self.attHistory = []     # List of (time, roll, pitch, yaw)
        self.gpsHistory = []     # List of (time, lat, lon)
        self.velHistory = []     # List of (time, x, y, z ,vx, vy, vz)
        self.linAccHistory = []  # List of (time, ax, ay, az)
        self.angHistory = []     # List of (time, wx, wy, wz)
        self.windHistory = []    # List of (time, wind_speed, wind_direction)
        self.simStateHistory = [] # List of (time, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon)
        
        
        self.lock = Lock()
        self._connect()
        self.thread = Thread(target=self._poll_data, daemon=True)
        self.thread.start()

    def _connect(self):
        self.conn = mavutil.mavlink_connection(f'udp:127.0.0.1:{self.udpPort}')
        print(f"[{self.name}] Waiting for heartbeat...")
        self.conn.wait_heartbeat()
        print(f"[{self.name}] Heartbeat received")

        # Request high-rate data
        self.conn.mav.request_data_stream_send(
            self.conn.target_system,
            self.conn.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            50, # Hz
            1 
        )

    def _poll_data(self):
        while True:
            msg = self.conn.recv_match(type=['LOCAL_POSITION_NED', 'ATTITUDE', 'GLOBAL_POSITION_INT', 'SCALED_IMU', 'WIND', 'SIMSTATE'], blocking=True)
            if msg is None:
                continue
            with self.lock:
                if msg.get_type() == 'LOCAL_POSITION_NED':
                    pos_time = msg.time_boot_ms / 1000
                    x = msg.x
                    y = msg.y
                    z = msg.z
                    vx = msg.vx
                    vy = msg.vy
                    vz = msg.vz
                    self.velHistory.append((pos_time, x, y, z, vx, vy, vz))
                    self.velHistory = self.velHistory[-100:]
                    self.posHistory.append((msg.x, msg.y))
                    self.posHistory = self.posHistory[-100:]
                elif msg.get_type() == 'ATTITUDE':
                    att_time = msg.time_boot_ms / 1000
                    roll = math.degrees(msg.roll)
                    pitch = math.degrees(msg.pitch)
                    yaw = math.degrees(msg.yaw)
                    self.attHistory.append((att_time, roll, pitch, yaw))
                    self.attHistory = self.attHistory[-100:]
                elif msg.get_type() == 'GLOBAL_POSITION_INT':
                    GPS_time = time.time()
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    self.gpsHistory.append((GPS_time, lat, lon))
                    self.gpsHistory = self.gpsHistory[-100:]
                elif msg.get_type() == 'SCALED_IMU':
                    imu_time = msg.time_usec / 1e6
                    ax = msg.xacc / 1000
                    ay = msg.yacc / 1000
                    az = msg.zacc / 1000
                    self.linAccHistory.append((imu_time, ax, ay, az))
                    self.linAccHistory = self.linAccHistory[-100:]

                    wx = msg.xgyro / 1000
                    wy = msg.ygyro / 1000
                    wz = msg.zgyro / 1000
                    self.angHistory.append((imu_time, wx, wy, wz))
                    self.angHistory = self.angHistory[-100:]
                elif msg.get_type() == 'WIND':
                    wind_time = msg.time_usec / 1e6
                    wind_speed = msg.speed / 100
                    wind_direction = math.degrees(msg.direction)
                    self.windHistory.append((wind_time, wind_speed, wind_direction))
                    self.windHistory = self.windHistory[-100:]
                elif msg.get_type() == 'SIMSTATE':
                    self.SIM_running = True
                    sim_time = time.time()
                    roll = math.degrees(msg.roll)
                    pitch = math.degrees(msg.pitch)
                    yaw = math.degrees(msg.yaw)
                    xacc = msg.xacc
                    yacc = msg.yacc
                    zacc = msg.zacc
                    xgyro = msg.xgyro
                    ygyro = msg.ygyro
                    zgyro = msg.zgyro
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    self.simStateHistory.append((sim_time, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon))
                    self.simStateHistory = self.simStateHistory[-100:]                
                
                

    def get_latest_data(self):
        with self.lock:
            return list(self.posHistory), list(self.attHistory), list(self.gpsHistory), list(self.velHistory), list(self.linAccHistory), list(self.angHistory), list(self.windHistory), list(self.simStateHistory)
        

    # def update(self):
    #     while True:
    #         msg = self.connection.recv_match(blocking=True)
    #         if msg is None:
    #             continue

    #         with self.lock:
    #             if msg.get_type() == "GLOBAL_POSITION_INT":
    #                 # self.latHistory.append(msg.lat / 1e7)
    #                 # self.lonHistory.append(msg.lon / 1e7)
    #                 # self.altHistory.append(msg.relative_alt / 1000)
    #             elif msg.get_type() == "RAW_IMU":
    #                 self.wxHistory.append(msg.xgyro / 1000)
    #                 self.wyHistory.append(msg.ygyro / 1000)
    #                 self.wzHistory.append(msg.zgyro / 1000)
    #             elif msg.get_type() == "LOCAL_POSITION_NED":
    #                 self.vxHistory.append(msg.vx / 100)
    #                 self.vyHistory.append(msg.vy / 100)
    #                 self.vzHistory.append(msg.vz / 100)

    #         time.sleep(0.01)
