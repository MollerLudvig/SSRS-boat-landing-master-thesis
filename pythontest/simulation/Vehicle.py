from pymavlink import mavutil
from subprocess import Popen, PIPE
from time import sleep
import numpy as np

class Vehicle():
    def __init__(self, connection):
        self.connection = connection

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
        return self.connection.recv_match(type=msg, blocking=False)
    
    def flush_messages(self):
        while True:
            msg = self.connection.recv_msg()
            if msg is None:
                break




    
