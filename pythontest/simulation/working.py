from pymavlink import mavutil
from subprocess import Popen, PIPE
from time import sleep
import numpy as np
import asyncio
from redis_communication import RedisClient
import WP_calculation as wp


R = 6371000 # Earth radius

Gr = 1/10 # Glide ratio
# NOTE: Can use descent_lookahead only for starting early (before P2), 
# then not look forward while already in descent
descent_lookahead = 8 # In meters, How far ahead in the slope the drone should look when deciding z_wanted
# "correct" descent_lookahead depends on Gr and impact speed: A farther P3 distance means less lookahead
# Ardupilot takes ~2-3 iterations until it starts descending properly and each iteration hase 1 sec delay
# So the descent can be up to 1 second late due to the delay aswell ----> 4 sec lookahead
cruise_altitude = 18 # In meters, 
aim_under_boat = 0 # In meters, If we want the drone to aim slightly under the boat
boat_length = 1.5 # In meters, Eyeballed length from drone that is driving boat to rear deck of boat
started_descent = False # Bool to keep track when descent is started


def tester(drone, boat):
    
    initiate_drone_settings(drone)
    drone_takeoff(drone)

    # Init redisclient
    rc = RedisClient()



    # Main loop
    while True:

        # Read redis stream for flight stage ("land", "follow")
        drone.stage =  rc.get_latest_stream_message("stage")[1]

        # Maybe be possible to set a desired_boat_speed to the same as drone speed and allow "straight down" landing
        # Exactly the same way current landing works, idk.
        catchup_speed = 3
        impact_speed = 2
        desired_boat_speed = drone.speed - impact_speed

        # Follow boat
        if drone.stage == "follow":

            # Calculate P2 and P1
            P2_distance = wp.calc_P2(drone.speed, desired_boat_speed, drone.altitude-boat.altitude+aim_under_boat, Gr)
            P2_lat, P2_lon = wp.calc_look_ahead_point(boat.lat, boat.lon, boat.heading-180, P2_distance)

            P1_distance = P2_distance + 20

            # Set boat's last recieved position as a waypoint for the drone
            target_lat, target_lon = wp.calc_look_ahead_point(P2_lat, P2_lon, boat.heading, 40)
            drone.follow_target([target_lat], [target_lon], [cruise_altitude-8])

            # Calculate the distance between drone and boat
            drone_distance_to_boat = wp.dist_between_coords(drone.lat, drone.lon, boat.lat, boat.lon) - boat_length
            # -boat_length because the actual landing spot is not where the boat coordinates are read

            print(f"P1 distance_ {P1_distance}")
            print(f"Drone distance: {drone_distance_to_boat}")

            # Change boat speed depending on how far the drone is 
            # since drone speed could not be changed directly in gazebo
            # CHANGE FOR REAL WORLD: Set drone speed and not boat speed
            if drone_distance_to_boat > P1_distance+10:
                boat.set_speed(drone.speed - catchup_speed)
            
            else:
                boat.set_speed(drone.speed)

            # Move boat
            boat_target_lat = boat.lat + 0.002
            boat_target_lon = boat.lon
            boat.set_guided_waypoint(boat_target_lat, boat_target_lon, 3)

        # Land on boat
        elif drone.stage == "land":
            
            # Move boat
            boat_target_lat = boat.lat + 0.002
            boat_target_lon = boat.lon
            boat.set_guided_waypoint(boat_target_lat, boat_target_lon, 3)

            # Calculate where P2 is
            P2_distance = wp.calc_P2(drone.speed, desired_boat_speed, drone.altitude-boat.altitude+aim_under_boat, Gr)
            P2_lat, P2_lon = wp.calc_look_ahead_point(boat.lat, boat.lon, boat.heading-180, P2_distance) # -180 because behind

            # Calculate drone distance to boat and boat distance to P3 (target)
            drone_distance_to_boat = wp.dist_between_coords(drone.lat, drone.lon, boat.lat, boat.lon) - boat_length
            boat_distance_to_target = wp.calc_landing_point_dist_boat(drone_distance_to_boat, drone.speed, desired_boat_speed)

            # Calculate where P3 (landing point) is
            P3_lat, P3_lon = wp.calc_look_ahead_point(boat.lat, boat.lon, boat.heading, boat_distance_to_target)

            # If drone is behind P2 + lookahead it should just keep flying towards the boat at cruise_altitude
            if (drone_distance_to_boat > P2_distance + descent_lookahead) and (not started_descent):
                boat.set_speed(drone.speed - catchup_speed)

                print(f"P2 distance: {P2_distance}")
                print(f"Drone distance: {drone_distance_to_boat}")
                print(f"Boat to target: {boat_distance_to_target}")
                print("\n")
                # Update prev_P3 and prev_Gr until we actually start landing
                prev_P3_lat, prev_P3_lon = P3_lat, P3_lon
                prev_Gr = Gr

                # Follow boat with some lookahead (30m) to smooth out the path
                boat_lh_lat, boat_lh_lon = wp.calc_look_ahead_point(boat.lat, boat.lon, boat.heading, 30)
                drone.follow_target([boat_lh_lat], [boat_lh_lon], [cruise_altitude-8])

            # When the drone passed P2 + lookahead it should start descending (landing)
            else:
                boat.set_speed(desired_boat_speed)
                # Calculate distance to previous iteration's P3, used along with prev_Gr to calculate z_wanted
                distance_to_prev_target = wp.dist_between_coords(drone.lat, drone.lon, prev_P3_lat, prev_P3_lon)

                # Calculate speed factor between relative and drone speed, 
                # needed to transform the lookahead from P2 (relative) frame to P3 (absolute) frame
                relative_speed = drone.speed - desired_boat_speed
                speed_factor = drone.speed/relative_speed

                # Calculate what altitude the drone should be at
                # Transform descent_lookahead to P3 frame
                # Add (boat_altitude - aim_under_boat) to account for not landing at 0m altitude
                # max() to no get negative z_wanted when the drone gets close to the boat
                z_wanted = (max(0, distance_to_prev_target - descent_lookahead*speed_factor - boat_length))*prev_Gr + boat.altitude - aim_under_boat

                # Fly towards P3 at boat's altitude (-8 because of drone's home position)
                drone.follow_target([P3_lat], [P3_lon], [boat.altitude-8])
                print(f"P2 distance: {P2_distance}")
                print(f"Drone distance: {drone_distance_to_boat}")
                print(f"Boat to target: {boat_distance_to_target}")
                print(f"Dist to prev: {distance_to_prev_target}")
                print("\n")

                # Calculate distance to new P3 and set prev_P3 to new P3
                distance_to_current_waypoint = wp.dist_between_coords(drone.lat, drone.lon, P3_lat, P3_lon)
                prev_P3_lat, prev_P3_lon = P3_lat, P3_lon

                # Update prev_Gr
                # Same as before, transform descent_lookahead to P3 frame
                # Add (boat_altitude-aim_under_boat)/prev_Gr to transform the altitude offset to a longitudinal distance
                new_Gr = z_wanted/(max(0, distance_to_current_waypoint - descent_lookahead*speed_factor - boat_length) + (boat.altitude - aim_under_boat)/prev_Gr)
                prev_Gr = new_Gr
                wanted_sink_rate = drone.speed*new_Gr

                # Calculate altitude error and a sink rate correction
                altitude_error = drone.altitude - z_wanted
                altitude_error_gain = 0.1
                sink_rate_correction = altitude_error*altitude_error_gain

                # Set sink rate of drone according to wanted sink rate and error correction
                drone.set_parameter("TECS_SINK_MIN", wanted_sink_rate + sink_rate_correction) # Default: 2.0
                drone.set_parameter("TECS_SINK_MAX", wanted_sink_rate + sink_rate_correction) # Default: 5.0

                print(f"z wanted: {z_wanted}")
                print(f"Gr: {new_Gr}")
                print(f"Altitude error: {altitude_error}")
                print(f"Wanted sink rate: {wanted_sink_rate}")
                print(f"Altitude: {drone.altitude}")
                print(f"Needed sink rate: {wanted_sink_rate + sink_rate_correction}")
                print("\n")
                started_descent = True
        
        # No command
        else:

            # Move boat
            boat_target_lat = boat.lat + 0.002
            boat_target_lon = boat.lon
            boat.set_speed(desired_boat_speed)
            boat.set_guided_waypoint(boat_target_lat, boat_target_lon, 3)

            # Loiter until command recieved
            drone.set_mode("GUIDED")
        
        # Flush all messages
        boat.flush_messages()
        drone.flush_messages()




def initiate_drone_settings(drone):
    # Parameter settings
    drone.set_parameter("TECS_SPDWEIGHT", 0.0) # Default: -1
    drone.set_parameter("TECS_TIME_CONST", 5.0) # Default: 5.0
    drone.set_parameter("TECS_THR_DAMP", 0.5) # Default: 0.5
    drone.set_parameter("TECS_PITCH_MIN", 0.0) # Default: 0.0
    drone.set_parameter("TECS_PTCH_DAMP", 0.3) # Default: 0.3
    drone.set_parameter("TECS_HGT_OMEGA", 3.0) # Default: 3.0
    drone.set_parameter("TECS_VERT_ACC", 7) # Default: 7
    drone.set_parameter("TECS_INTEG_GAIN", 0.3) # Default: 0.3

    sleep(5)


async def drone_takeoff(drone):
     # Set modes
    drone.set_mode("FBWB")
    
    # Arm vehicles
    drone.arm_vehicle()
    sleep(5)

    # Takeoff vehicles
    drone.set_servo(3, 1800)
    sleep(5)

async def boat_launch(boat):
    # Set modes
    boat.set_mode("GUIDED")
    
    # Arm vehicles
    boat.arm_vehicle()
    sleep(5)

    # Takeoff vehicles
    boat.takeoff(3)
    sleep(5)