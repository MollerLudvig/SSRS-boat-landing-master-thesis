from pymavlink import mavutil
from subprocess import Popen, PIPE
from time import sleep
import numpy as np
from redis_communication import RedisClient

from Drone import Drone
from Boat import Boat

R = 6371000 # Earth radius

def set_parameter(connection, param_name, value):
    """Sets an ArduPilot parameter via MAVLink"""
    connection.mav.param_set_send(
        connection.target_system, connection.target_component,
        param_name.encode(),  # Encode parameter name
        float(value),         # Parameter value
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32  # Data type
    )

# Calculate distance between coordinates
def dist_between_coords(lat1, lon1, lat2, lon2):
    lat1_rad = np.deg2rad(lat1)
    lon1_rad = np.deg2rad(lon1)
    lat2_rad = np.deg2rad(lat2)
    lon2_rad = np.deg2rad(lon2)

    delta_lat = lat2_rad-lat1_rad
    delta_lon = lon2_rad-lon1_rad
    mean_lat = (lat1_rad+lat2_rad)/2
    d = np.sqrt((delta_lat*R)**2 + (delta_lon*R*np.cos(mean_lat))**2)
    return d

# Calculate coordinates a distance in heading
def calc_look_ahead_point(lat, lon, heading, look_ahead_dist):

    heading_rad = np.radians(heading)

    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    # Spherical geometry
    target_lat_rad = lat_rad + (look_ahead_dist / R) * np.cos(heading_rad)
    target_lon_rad = lon_rad + (look_ahead_dist / (R * np.cos(lat_rad))) * np.sin(heading_rad)

    target_lat = np.degrees(target_lat_rad)
    target_lon = np.degrees(target_lon_rad)

    return target_lat, target_lon

# Calculate distance to landing point
def calc_landing_point_dist_drone(d, drone_speed, boat_speed):
    relative_speed = drone_speed - boat_speed
    time_to_catch_up = d/relative_speed
    waypoint_distance = drone_speed*time_to_catch_up
    return waypoint_distance

def calc_landing_point_dist_boat(d, drone_speed, boat_speed):
    relative_speed = drone_speed - boat_speed
    time_to_catch_up = d/relative_speed
    waypoint_distance = boat_speed*time_to_catch_up
    return waypoint_distance

def calc_P2(drone_speed, boat_speed, altitude, glide_slope):
    drone_intercept_distance = altitude/glide_slope
    time_to_intercept = drone_intercept_distance/drone_speed
    boat_intercept_distance = boat_speed*time_to_intercept
    P2_distance = drone_intercept_distance - boat_intercept_distance
    return P2_distance

def tester():

    # Establish connection
    drone_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    boat_connection = mavutil.mavlink_connection('udp:127.0.0.1:14560')

    drone_connection.wait_heartbeat()
    boat_connection.wait_heartbeat()

    drone = Drone(drone_connection)
    boat = Boat(boat_connection)

    drone_msg = drone.get_message('HEARTBEAT')
    boat_msg = boat.get_message('HEARTBEAT')
    
    sleep(5)
    set_parameter(drone_connection, "TECS_SPDWEIGHT", 0.0) # Default: -1
    set_parameter(drone_connection, "TECS_TIME_CONST", 3.0) # Default: 5.0
    # set_parameter(drone_connection, "TECS_SINK_MIN", 2.0) # Default: 2.0
    # set_parameter(drone_connection, "TECS_SINK_MAX", 5.0) # Default: 5.0
    set_parameter(drone_connection, "TECS_THR_DAMP", 0.5) # Default: 0.5
    set_parameter(drone_connection, "TECS_PITCH_MIN", 0.0) # Default: 0.0
    set_parameter(drone_connection, "TECS_PTCH_DAMP", 0.3) # Default: 0.3
    set_parameter(drone_connection, "TECS_HGT_OMEGA", 5.0) # Default: 3.0
    set_parameter(drone_connection, "TECS_VERT_ACC", 7) # Default: 7
    set_parameter(drone_connection, "TECS_INTEG_GAIN", 0.3) # Default: 0.3

    # Set modes
    rc = RedisClient()

    drone.set_mode("FBWB")
    boat.set_mode("GUIDED")
    
    # Arm vehicles
    drone.arm_vehicle()
    sleep(3)
    boat.arm_vehicle()
    sleep(5)

    # Takeoff vehicles
    boat.takeoff(3)
    sleep(10)
    drone.set_servo(3, 1800)
    sleep(3)

    Gr = 1/10
    descent_lookahead = 20 # meters rn - How far ahead in the glide slope the drone should look when calculating z_wanted
    aim_under_boat = 1 # meters - How far under the boat the landing point should be put
    cruise_altitude = 18
    boat_length = 3 # meters - estimate just from looking 
    started_descent = False

    # Main loop
    i = 0
    while True:

        while True:
            boat_msg = boat.get_message('GLOBAL_POSITION_INT')
            drone_msg = drone.get_message('GLOBAL_POSITION_INT')
            if boat_msg and drone_msg:
                break

        # Retrieve all data
        boat_lat = boat_msg.lat / 1e7
        boat_lon = boat_msg.lon / 1e7
        boat_heading = boat_msg.hdg /100
        boat_vx = boat_msg.vx / 100
        boat_vy = boat_msg.vy /100
        boat_speed = np.sqrt(boat_vx**2+boat_vy**2)
        boat_altitude = boat_msg.alt/1000

        drone_lat = drone_msg.lat / 1e7
        drone_lon = drone_msg.lon / 1e7
        drone_heading = drone_msg.hdg / 100
        drone_vx = drone_msg.vx / 100
        drone_vy = drone_msg.vy /100
        drone_speed = np.sqrt(drone_vx**2+drone_vy**2)
        drone_altitude = drone_msg.alt/1000
        # Maybe put these as attributes to the drone and boat classes

        # Read redis stream for flight stage ("land", "follow")
        drone.stage =  rc.get_latest_stream_message("stage")[1]

        # Maybe be possible to set a desired_boat_speed to the same as drone speed and allow "straight down" landing
        # Exactly the same way current landing works, idk.
        desired_boat_speed = 13

        # Follow boat
        if drone.stage == "follow":

            # Calculate P2 and P3
            print(f"Drone altitude: {drone_altitude}")
            P2_distance = calc_P2(drone_speed, desired_boat_speed, (drone_altitude-boat_altitude+aim_under_boat), Gr) # boat is not at sea level
            P2_lat, P2_lon = calc_look_ahead_point(boat_lat, boat_lon, boat_heading-180, P2_distance)

            P1_distance = P2_distance + 20

            # Follow 
            target_lat, target_lon = calc_look_ahead_point(P2_lat, P2_lon, boat_heading, 40)
            drone.follow_target([target_lat], [target_lon], [cruise_altitude])

            drone_distance_to_boat = dist_between_coords(drone_lat, drone_lon, boat_lat, boat_lon)
            print(f"P1 distance_ {P1_distance}")
            print(f"Drone distance: {drone_distance_to_boat}")

            if drone_distance_to_boat > P1_distance+10:
                boat.set_speed(desired_boat_speed)
            
            else:
                boat.set_speed(drone_speed)

            # Move boat
            boat_target_lat = boat_lat + 0.002
            boat_target_lon = boat_lon
            boat.set_guided_waypoint(boat_target_lat, boat_target_lon, 3)

        # Land on boat
        elif drone.stage == "land":
            
            # Move boat
            boat_target_lat = boat_lat + 0.002
            boat_target_lon = boat_lon
            boat.set_guided_waypoint(boat_target_lat, boat_target_lon, 3)

            P2_distance = calc_P2(drone_speed, desired_boat_speed, drone_altitude-boat_altitude+aim_under_boat, Gr)
            P2_lat, P2_lon = calc_look_ahead_point(boat_lat, boat_lon, boat_heading-180, P2_distance)

            drone_distance_to_boat = dist_between_coords(drone_lat, drone_lon, boat_lat, boat_lon) - boat_length
            boat_distance_to_target = calc_landing_point_dist_boat(drone_distance_to_boat, drone_speed, desired_boat_speed)

            P3_lat, P3_lon = calc_look_ahead_point(boat_lat, boat_lon, boat_heading, boat_distance_to_target) #<------- Target waypoint

            if (drone_distance_to_boat > P2_distance) and (not started_descent):
                prev_P3_lat,prev_P3_lon = P3_lat, P3_lon
                prev_Gr = Gr
                boat_lh_lat, boat_lh_lon = calc_look_ahead_point(boat_lat, boat_lon, boat_heading, 30)
                drone.follow_target([boat_lh_lat], [boat_lh_lon], [cruise_altitude])
                boat.set_speed(desired_boat_speed)

            else:
                boat.set_speed(desired_boat_speed)
                
                distance_to_prev_target = dist_between_coords(drone_lat, drone_lon, prev_P3_lat, prev_P3_lon)
                
                z_wanted = (distance_to_prev_target-descent_lookahead)*prev_Gr # - descent_lookahead is lookahead to go to an altitude we should be at farther in the future
                z_target = boat_altitude - aim_under_boat 
                # Prob need some adaptive look ahead distance depending on Gr
                print(f"prev_Gr: {prev_Gr}")

                drone.follow_target([P3_lat], [P3_lon], [min(cruise_altitude, z_target)]) 

                print(f"Distance to previous waypoint: {int(distance_to_prev_target)}")
                print(f"Target altitude: {int(z_wanted)}")
                print(f"Drone altitude: {drone_altitude}")

                distance_to_current_waypoint = dist_between_coords(drone_lat, drone_lon, P3_lat, P3_lon)

                print(f"Distance to current waypoint: {int(distance_to_current_waypoint)}")
                prev_P3_lat, prev_P3_lon = P3_lat, P3_lon

                new_Gr = z_wanted/(distance_to_current_waypoint-descent_lookahead)
                wanted_sink_rate = drone_speed*new_Gr
                needed_sink_rate = wanted_sink_rate + (drone_altitude-z_wanted)*0.5
                set_parameter(drone_connection, "TECS_SINK_MIN", 2) # Default: 2.0
                set_parameter(drone_connection, "TECS_SINK_MAX", 5) # Default: 5.0
                print(f"Needed descent rate: {needed_sink_rate}")
                print("\n")

                prev_Gr = new_Gr

                started_descent = True

        
        # No command
        else:
            # Move boat
            boat_target_lat = boat_lat + 0.002
            boat_target_lon = boat_lon
            boat.set_speed(13)
            boat.set_guided_waypoint(boat_target_lat, boat_target_lon, 3)
            drone.set_mode("GUIDED")

        i += 1

        boat.flush_messages()
        drone.flush_messages()







