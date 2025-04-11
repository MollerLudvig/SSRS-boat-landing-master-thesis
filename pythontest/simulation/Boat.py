from pymavlink import mavutil
from subprocess import Popen, PIPE
from time import sleep
import numpy as np
from Vehicle import Vehicle

class Boat(Vehicle):
    def __init__(self, connection):
        self.connection = connection
        self.deck_lat = None
        self.deck_lon = None
        super().__init__(connection)


    def set_guided_waypoint(self, lat, lon, alt):
        # Send a new target position in GUIDED mode
        bmsg1 = self.connection.mav.mission_item_int_send(
            self.connection.target_system,  # Target system ID (0 for broadcast)
            self.connection.target_component,  # Target component ID (0 for broadcast)
            0,  # Sequence number (1 for the first waypoint)
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # Frame of reference
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # Command ID for waypoint
            2,  # Current (set to 0 for new waypoints)
            1,  # Autocontinue (set to 0 for new waypoints)
            0,  # Param 1 (not used for waypoints)
            0,  # Param 2 (not used for waypoints)
            0,  # Param 3 (not used for waypoints)
            0,  # Param 4 (not used for waypoints)
            int(lat * 1e7),  # Param 5 (latitude in degrees * 1e7)
            int(lon * 1e7),  # Param 6 (longitude in degrees * 1e7)
            int(alt),  # Param 7 (altitude in meters)
            0
        )

    def takeoff(self, alt):
        self.connection.mav.command_long_send(
        self.connection.target_system, self.connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, alt
    )
    





