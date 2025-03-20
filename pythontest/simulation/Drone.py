from pymavlink import mavutil
from subprocess import Popen, PIPE
from time import sleep
import numpy as np

from Vehicle import Vehicle

class Drone(Vehicle):
    def __init__(self, connection):
        self.connection = connection
        self.stage = None
        super().__init__(connection)

    def upload_mission(self, waypoints):
        # Clear existing mission
        self.connection.mav.mission_clear_all_send(self.connection.target_system, self.connection.target_component)

        # Send mission count
        self.connection.mav.mission_count_send(self.connection.target_system, self.connection.target_component, len(waypoints))

        for i, (lat, lon, alt) in enumerate(waypoints):
            msg = mavutil.mavlink.MAVLink_mission_item_int_message(
                target_system=self.connection.target_system,
                target_component=self.connection.target_component,
                seq=i,
                frame=mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                command=mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                current=0,
                autocontinue=1,
                param1=0, param2=5, param3=0, param4=0,
                x=int(lat * 1e7),
                y=int(lon * 1e7),
                z=int(alt),
                mission_type=0
            )
            self.connection.mav.send(msg)

    def follow_target(self, lat, lon, alt):
        waypoint1 = (lat[0], lon[0], alt[0])
        #waypoint2 = (lat, lon, alt)

        self.upload_mission([waypoint1, waypoint1])
        
        self.set_mode("AUTO")
        self.set_current_waypoint(1)
    
    def set_servo(self, channel, pwm):
        self.connection.mav.command_long_send(
            self.connection.target_system, self.connection.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, channel, pwm, 0, 0, 0, 0, 0
        )
    def set_current_waypoint(self, waypoint_index):
        self.connection.mav.mission_set_current_send(
            self.connection.target_system,
            self.connection.target_component,
            waypoint_index
        )
        print(f"Set current waypoint to {waypoint_index}")




    
