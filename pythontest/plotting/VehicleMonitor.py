from pymavlink import mavutil
from threading import Thread, Lock
import math
import time
import argparse

save_history = 100

class VehicleMonitor:
    def __init__(self, connection_string, save_CSV=True, color='blue', 
                 enableDroneWindow=True, enableBoatWindow=True, enableGlobalWindow=True):
        self.SIM_running = False
        self.save_CSV = save_CSV
        self.connection_string = connection_string
        self.color = color

        self.enableDroneWindow = enableDroneWindow
        self.enableBoatWindow = enableBoatWindow
        self.enableGlobalWindow = enableGlobalWindow

        self.posHistory = []     # List of (time, x, y, z, vx, vy, vz)
        self.attHistory = []     # List of (time, roll, pitch, yaw)
        self.gpsHistory = []     # List of (time, lat, lon)
        self.velHistory = []     # List of (time, x, y, z, vx, vy, vz)
        self.linAccHistory = []  # List of (time, ax, ay, az)
        self.angHistory = []     # List of (time, wx, wy, wz)
        self.windHistory = []    # List of (time, wind_speed, wind_direction)
        self.simStateHistory = [] # List of (time, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon)
        
        self.is_connected = False
        self.lock = Lock()
        self.conn = None
        
    def connect(self):
        """Connect to the vehicle and start polling data"""
        self._connect()
        if self.is_connected:
            self.thread = Thread(target=self._poll_data, daemon=True)
            self.thread.start()
            return True
        return False

    def _connect(self):
        """Establish connection to the vehicle"""
        try:
            print(f"Connecting to vehicle via {self.connection_string}...")
            self.conn = mavutil.mavlink_connection(self.connection_string)
            print("Waiting for heartbeat...")
            self.conn.wait_heartbeat(timeout=10)
            print(f"Heartbeat received from system (system {self.conn.target_system}, component {self.conn.target_component})")
            
            # Request high-rate data
            self.conn.mav.request_data_stream_send(
                self.conn.target_system,
                self.conn.target_component,
                mavutil.mavlink.MAV_DATA_STREAM_ALL,
                50,  # Hz
                1 
            )
            
            self.is_connected = True
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            self.is_connected = False
            return False

    def _poll_data(self):
        """Poll for and process MAVLink messages"""
        while self.is_connected:
            try:
                msg = self.conn.recv_match(
                    type=['LOCAL_POSITION_NED', 'ATTITUDE', 'GLOBAL_POSITION_INT', 'SCALED_IMU', 'WIND', 'SIM_STATE'], 
                    blocking=True, 
                    timeout=1.0
                )
                
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
                        self.posHistory.append((pos_time, x, y, z, vx, vy, vz))
                        self.posHistory = self.posHistory[-save_history:]
                    elif msg.get_type() == 'ATTITUDE':
                        att_time = msg.time_boot_ms / 1000
                        roll = math.degrees(msg.roll)
                        pitch = math.degrees(msg.pitch)
                        yaw = math.degrees(msg.yaw)
                        roll_speed = math.degrees(msg.rollspeed)
                        pitch_speed = math.degrees(msg.pitchspeed)
                        yaw_speed = math.degrees(msg.yawspeed)
                        self.attHistory.append((att_time, roll, pitch, yaw, roll_speed, pitch_speed, yaw_speed))
                        self.attHistory = self.attHistory[-save_history:]
                    elif msg.get_type() == 'GLOBAL_POSITION_INT':
                        GPS_time = msg.time_boot_ms / 1000
                        lat = msg.lat / 1e7
                        lon = msg.lon / 1e7
                        alt = msg.alt / 1000
                        relative_alt = msg.relative_alt / 1000
                        vx = msg.vx / 100
                        vy = msg.vy / 100
                        vz = msg.vz / 100
                        hdg = msg.hdg / 100
                        self.gpsHistory.append((GPS_time, lat, lon, alt, relative_alt, vx, vy, vz, hdg))
                        self.gpsHistory = self.gpsHistory[-save_history:]
                    elif msg.get_type() == 'SCALED_IMU':
                        imu_time = msg.time_boot_ms / 1000
                        ax = msg.xacc / 1000
                        ay = msg.yacc / 1000
                        az = msg.zacc / 1000
                        self.linAccHistory.append((imu_time, ax, ay, az))
                        self.linAccHistory = self.linAccHistory[-save_history:]

                        wx = msg.xgyro / 1000
                        wy = msg.ygyro / 1000
                        wz = msg.zgyro / 1000
                        self.angHistory.append((imu_time, wx, wy, wz))
                        self.angHistory = self.angHistory[-save_history:]
                    elif msg.get_type() == 'WIND':
                        wind_time = time.time()  # Time not provided in message
                        wind_speed = msg.speed
                        wind_direction = msg.direction
                        self.windHistory.append((wind_time, wind_speed, wind_direction))
                        self.windHistory = self.windHistory[-save_history:]
                    elif msg.get_type() == 'SIM_STATE':
                        self.SIM_running = True
                        sim_time = time.time()  # Time not provided in message
                        roll = math.degrees(msg.roll)
                        pitch = math.degrees(msg.pitch)
                        yaw = math.degrees(msg.yaw)
                        xacc = msg.xacc
                        yacc = msg.yacc
                        zacc = msg.zacc
                        xgyro = math.degrees(msg.xgyro)
                        ygyro = math.degrees(msg.ygyro)
                        zgyro = math.degrees(msg.zgyro)
                        lat = msg.lat_int / 1e7
                        lon = msg.lon_int / 1e7
                        alt = msg.alt
                        vn = msg.vn
                        ve = msg.ve
                        vd = msg.vd
                        self.simStateHistory.append((sim_time, roll, pitch, yaw, xacc, yacc, zacc, xgyro, ygyro, zgyro, lat, lon, alt, vn, ve, vd))
                        self.simStateHistory = self.simStateHistory[-save_history:]
            except Exception as e:
                print(f"Error in polling: {e}")
                time.sleep(1)

    def get_latest_data(self):
        """Get the latest data from all telemetry sources"""
        with self.lock:
            return list(self.posHistory), list(self.attHistory), list(self.gpsHistory), list(self.velHistory), list(self.linAccHistory), list(self.angHistory), list(self.windHistory), list(self.simStateHistory)
        
    def close(self):
        """Close the connection"""
        self.is_connected = False
        if self.conn:
            self.conn.close()
            print("Connection closed")


# Example usage
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Connect to vehicle via MAVLink')
    parser.add_argument('--connection', default='serial:/dev/ttyUSB0:57600', 
                      help='Connection string (default: serial:/dev/ttyUSB0:57600)')
    args = parser.parse_args()
    
    # Create vehicle monitor
    vehicle = VehicleMonitor(args.connection)
    
    if vehicle.connect():
        try:
            print("Connected successfully! Press Ctrl+C to exit.")
            
            # Display data example
            while True:
                if vehicle.gpsHistory:
                    latest_gps = vehicle.gpsHistory[-1]
                    print(f"GPS: Lat={latest_gps[1]:.6f}, Lon={latest_gps[2]:.6f}, Alt={latest_gps[3]:.1f}m")
                
                if vehicle.attHistory:
                    latest_att = vehicle.attHistory[-1]
                    print(f"Attitude: Roll={latest_att[1]:.1f}°, Pitch={latest_att[2]:.1f}°, Yaw={latest_att[3]:.1f}°")
                
                time.sleep(1)
                
        except KeyboardInterrupt:
            print("\nExiting...")
        finally:
            vehicle.close()
    else:
        print("Failed to connect to vehicle.")