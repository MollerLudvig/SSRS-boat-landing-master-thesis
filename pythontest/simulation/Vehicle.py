from pymavlink import mavutil
from subprocess import Popen, PIPE
from time import sleep
import numpy as np

class Vehicle():
    def __init__(self, connection):
        self.connection = connection
        self.lat = None
        self.lon = None
        self.heading = None
        self.vx = None
        self.vy = None
        self.vz = None
        self.speed = None
        self.altitude = None
        self.data = {}

        # Filter variables
        self.inital_lat = None
        self.inital_lon = None
        self.x = None
        self.xdot = None
        self.y = None
        self.ydot = None

    def set_parameter(self, param_name, value):
        """Sets an ArduPilot parameter via MAVLink"""
        self.connection.mav.param_set_send(
            self.connection.target_system, self.connection.target_component,
            param_name.encode(),  # Encode parameter name
            float(value),         # Parameter value
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32  # Data type
        )

    def arm_vehicle(self):
        while not self.connection.motors_armed():
            self.connection.mav.command_long_send(
                self.connection.target_system, self.connection.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
            )

            msg = self.get_message('HEARTBEAT')

            print("trying to arm drone")
            sleep(1)
        print("drone is armed")

    def set_mode(self, mode):
        mode_id = self.connection.mode_mapping().get(mode) 

        while True:
            self.connection.mav.set_mode_send(
                self.connection.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                mode_id
            )
            msg = self.get_message('HEARTBEAT')
            if msg and msg.custom_mode == mode_id or mode_id:
                return True
            sleep(1)
    def set_speed(self, speed):
        """Set the vehicle's target speed."""
        speed_msg = self.connection.mav.command_long_encode(
            self.connection.target_system,  # Target system ID
            self.connection.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # Command ID for changing speed
            0,  # Confirmation
            0,  # Speed type: 0 = airspeed, 1 = ground speed
            speed,  # Target speed (m/s)
            -1,  # Throttle (ignored)
            0, 0, 0, 0  # Unused parameters
        )
        self.connection.mav.send(speed_msg)

    def get_message(self, msg):
        return self.connection.recv_match(type=msg, blocking=True)
    
    def flush_messages(self):
        while True:
            msg = self.connection.recv_msg()
            if msg is None:
                break

    def get_lat(self):
        return 

    def update_possition_mavlink(self):
        pos_msg = self.get_message('GLOBAL_POSITION_INT')
        self.lat = pos_msg.lat / 1e7
        self.lon = pos_msg.lon / 1e7
        self.heading = pos_msg.hdg / 100
        self.vx = pos_msg.vx / 100
        self.vy = pos_msg.vy / 100
        self.vz = pos_msg.vz / 100
        self.speed = np.sqrt(self.vx**2+self.vy**2)
        self.altitude = pos_msg.alt/1000


    
