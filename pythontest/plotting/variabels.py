from VehicleMonitor import VehicleMonitor

from dataclasses import dataclass, field
from typing import List, Optional, Tuple
from threading import Lock
import copy


@dataclass
class PositionData:
    time: Optional[List[float]] = None
    x: Optional[List[float]] = None
    y: Optional[List[float]] = None
    z: Optional[List[float]] = None
    vx: Optional[List[float]] = None
    vy: Optional[List[float]] = None
    vz: Optional[List[float]] = None


@dataclass
class AttitudeData:
    time: Optional[List[float]] = None
    roll: Optional[List[float]] = None
    pitch: Optional[List[float]] = None
    yaw: Optional[List[float]] = None
    roll_speed: Optional[List[float]] = None
    pitch_speed: Optional[List[float]] = None
    yaw_speed: Optional[List[float]] = None


@dataclass
class GPSData:
    time: Optional[List[float]] = None
    lat: Optional[List[float]] = None
    lon: Optional[List[float]] = None
    alt: Optional[List[float]] = None
    relative_alt: Optional[List[float]] = None
    vx: Optional[List[float]] = None
    vy: Optional[List[float]] = None
    vz: Optional[List[float]] = None
    hdg: Optional[List[float]] = None


@dataclass
class IMUData:
    time: Optional[List[float]] = None
    # Accelerometer data
    ax: Optional[List[float]] = None
    ay: Optional[List[float]] = None
    az: Optional[List[float]] = None
    # Gyroscope data
    wx: Optional[List[float]] = None
    wy: Optional[List[float]] = None
    wz: Optional[List[float]] = None


@dataclass
class WindData:
    time: Optional[List[float]] = None
    speed: Optional[List[float]] = None
    direction: Optional[List[float]] = None


@dataclass
class SimulationData:
    time: Optional[List[float]] = None
    roll: Optional[List[float]] = None
    pitch: Optional[List[float]] = None
    yaw: Optional[List[float]] = None
    xacc: Optional[List[float]] = None
    yacc: Optional[List[float]] = None
    zacc: Optional[List[float]] = None
    xgyro: Optional[List[float]] = None
    ygyro: Optional[List[float]] = None
    zgyro: Optional[List[float]] = None
    lat: Optional[List[float]] = None
    lon: Optional[List[float]] = None
    alt: Optional[List[float]] = None
    vn: Optional[List[float]] = None  # North velocity
    ve: Optional[List[float]] = None  # East velocity
    vd: Optional[List[float]] = None  # Down velocity
    vx: Optional[List[float]] = None
    vy: Optional[List[float]] = None
    vz: Optional[List[float]] = None

class VehicleData:
    """
    Structured container for vehicle data that safely stores data from VehicleMonitor.
    This implementation groups related data into dataclasses for better organization
    and maintainability. It addresses thread safety concerns by providing a single
    update method that atomically updates all data.
    
    This class is specifically designed to address thread safety issues in VehicleMonitor,
    where direct access to the history variables could lead to inconsistent data due to
    concurrent modifications by the data polling thread.
    """
    def __init__(self, vehicle_monitor):
        self.vehicle_monitor = vehicle_monitor
        self.position = PositionData()
        self.attitude = AttitudeData()
        self.gps = GPSData()
        self.imu = IMUData()
        self.wind = WindData()
        self.simulation = SimulationData()
        self._lock = Lock()  # Add our own lock for thread safety
        self.distance_to_tail = []
        self.distance_to_tail_time = []

    def update(self):
        """
        Fetch and update all vehicle data from the monitor in one operation
        to maintain data consistency.
        
        This method safely retrieves all data from VehicleMonitor at once using
        its thread-safe get_latest_data() method, then updates our local copies
        under our own lock to ensure consistent reads from this object.
        """
        # First get the data using VehicleMonitor's thread-safe method
        pos, att, gps, vel, acc, ang, wind, sim = self.vehicle_monitor.get_latest_data()
        
        # Now update our local copies under our own lock
        with self._lock:
            if pos:
                # Unpack data using zip
                times, xs, ys, zs, vxs, vys, vzs = zip(*pos)
                self.position.time = list(times)
                self.position.x = list(xs)
                self.position.y = list(ys)
                self.position.z = list(zs)
                self.position.vx = list(vxs)
                self.position.vy = list(vys)
                self.position.vz = list(vzs)

            if att:
                times, rolls, pitches, yaws, roll_speeds, pitch_speeds, yaw_speeds = zip(*att)
                self.attitude.time = list(times)
                self.attitude.roll = list(rolls)
                self.attitude.pitch = list(pitches)
                self.attitude.yaw = list(yaws)
                self.attitude.roll_speed = list(roll_speeds)
                self.attitude.pitch_speed = list(pitch_speeds)
                self.attitude.yaw_speed = list(yaw_speeds)

            if gps:
                times, lats, lons, alts, rel_alts, vxs, vys, vzs, hdgs = zip(*gps)
                self.gps.time = list(times)
                self.gps.lat = list(lats)
                self.gps.lon = list(lons)
                self.gps.alt = list(alts)
                self.gps.relative_alt = list(rel_alts)
                self.gps.vx = list(vxs)
                self.gps.vy = list(vys)
                self.gps.vz = list(vzs)
                self.gps.hdg = list(hdgs)

            if acc and ang:
                # Combining accelerometer and gyroscope data
                acc_times, axs, ays, azs = zip(*acc)
                ang_times, wxs, wys, wzs = zip(*ang)
                
                # Assuming both have the same timestamps, otherwise you might need more complex logic
                self.imu.time = list(acc_times)
                self.imu.ax = list(axs)
                self.imu.ay = list(ays)
                self.imu.az = list(azs)
                self.imu.wx = list(wxs)
                self.imu.wy = list(wys)
                self.imu.wz = list(wzs)
            
            if wind:
                times, speeds, directions = zip(*wind)
                self.wind.time = list(times)
                self.wind.speed = list(speeds)
                self.wind.direction = list(directions)

            if sim:
                # Unpack simulation data
                (times, rolls, pitches, yaws, xaccs, yaccs, zaccs, 
                xgyros, ygyros, zgyros, lats, lons, alts, 
                vns, ves, vds) = zip(*sim)
                
                self.simulation.time = list(times)
                self.simulation.roll = list(rolls)
                self.simulation.pitch = list(pitches)
                self.simulation.yaw = list(yaws)
                self.simulation.xacc = list(xaccs)
                self.simulation.yacc = list(yaccs)
                self.simulation.zacc = list(zaccs)
                self.simulation.xgyro = list(xgyros)
                self.simulation.ygyro = list(ygyros)
                self.simulation.zgyro = list(zgyros)
                self.simulation.lat = list(lats)
                self.simulation.lon = list(lons)
                self.simulation.alt = list(alts)
                self.simulation.vn = list(vns)
                self.simulation.ve = list(ves)
                self.simulation.vd = list(vds)

    def get_latest_position(self):
        """Return the most recent position data point in a thread-safe manner"""
        with self._lock:
            if not self.position.time:
                return None
            idx = -1  # Last element
            return (
                self.position.time[idx],
                self.position.x[idx],
                self.position.y[idx],
                self.position.z[idx],
                self.position.vx[idx],
                self.position.vy[idx],
                self.position.vz[idx],
            )
            
    def get_latest_attitude(self):
        """Return the most recent attitude data point in a thread-safe manner"""
        with self._lock:
            if not self.attitude.time:
                return None
            idx = -1
            return (
                self.attitude.time[idx],
                self.attitude.roll[idx],
                self.attitude.pitch[idx],
                self.attitude.yaw[idx],
                self.attitude.roll_speed[idx],
                self.attitude.pitch_speed[idx],
                self.attitude.yaw_speed[idx],
            )
            
    def get_latest_gps(self):
        """Return the most recent GPS data point in a thread-safe manner"""
        with self._lock:
            if not self.gps.time:
                return None
            idx = -1
            return (
                self.gps.time[idx],
                self.gps.lat[idx],
                self.gps.lon[idx],
                self.gps.alt[idx],
                self.gps.relative_alt[idx],
                self.gps.vx[idx],
                self.gps.vy[idx],
                self.gps.vz[idx],
                self.gps.hdg[idx],
            )
            
    def get_snapshot(self):
        """
        Return a complete snapshot of all current data in a thread-safe manner.
        This creates deep copies of all data to ensure thread safety.
        """
        with self._lock:
            snapshot = {}
            snapshot['position'] = copy.deepcopy(self.position)
            snapshot['attitude'] = copy.deepcopy(self.attitude)
            snapshot['gps'] = copy.deepcopy(self.gps)
            snapshot['imu'] = copy.deepcopy(self.imu)
            snapshot['wind'] = copy.deepcopy(self.wind)
            snapshot['simulation'] = copy.deepcopy(self.simulation)
            return snapshot
