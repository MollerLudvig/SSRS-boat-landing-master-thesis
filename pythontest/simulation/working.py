from pymavlink import mavutil
from subprocess import Popen, PIPE
from time import sleep
import time
import numpy as np
from redis_communication import RedisClient
import WP_calculation as wp
import matplotlib.pyplot as plt

from Drone import Drone
from Boat import Boat

R = 6371000 # Earth radius

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
    
    # Parameter settings
    drone.set_parameter("TECS_SPDWEIGHT", 0.0) # Default: -1
    drone.set_parameter("TECS_TIME_CONST", 3.0) # Default: 5.0
    drone.set_parameter("TECS_THR_DAMP", 0.5) # Default: 0.5
    drone.set_parameter("TECS_PITCH_MIN", 0.0) # Default: 0.0
    drone.set_parameter("TECS_PTCH_DAMP", 0.3) # Default: 0.3
    drone.set_parameter("TECS_HGT_OMEGA", 3.0) # Default: 3.0
    drone.set_parameter("TECS_VERT_ACC", 8) # Default: 7
    drone.set_parameter("TECS_INTEG_GAIN", 0.5) # Default: 0.3

    sleep(5)
    # Init redisclient
    rc = RedisClient()

    # Set modes
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

    Gr = 1/10 # Glide ratio
    # For missionhandler abort condition
    rc.add_stream_message("needed_glide_ratio", Gr)
    
    # NOTE: Can use descent_lookahead only for starting early (before P2), 
    # then not look forward while already in descent
    descent_lookahead = 3 # In meters, How far ahead in the slope the drone should look when deciding z_wanted
    # "correct" descent_lookahead depends on Gr and impact speed: A farther P3 distance means less lookahead
    # Ardupilot takes ~2-3 iterations until it starts descending properly and each iteration hase 1 sec delay
    # So the descent can be up to 1 second late due to the delay aswell ----> 4 sec lookahead

    cruise_altitude = 18 # In meters, 
    aim_under_boat = 0 # In meters, If we want the drone to aim slightly under the boat
    boat_length = 2.5 # In meters, Eyeballed length from drone that is driving boat to rear deck of boat
    altitude_error_gain = 0.225
    started_descent = False # Bool to keep track when descent is started

    drone_altitudes = []
    drone_coordinates = []
    z_wanteds = []
    needed_sink_rates = []
    wanted_sink_rates = []
    xs = [0]

    i = 0
    # Main loop
    while True:

        # Retrieve all data
        boat_pos_msg = boat.get_message('GLOBAL_POSITION_INT')
        boat.lat = boat_pos_msg.lat / 1e7
        boat.lon = boat_pos_msg.lon / 1e7
        boat.heading = boat_pos_msg.hdg /100
        boat.deck_lat, boat.deck_lon = wp.calc_look_ahead_point(boat.lat, boat.lon, boat.heading-180, boat_length)

        boat.vx = boat_pos_msg.vx / 100
        boat.vy = boat_pos_msg.vy /100
        boat.speed = np.sqrt(boat.vx**2+boat.vy**2)
        boat.altitude = boat_pos_msg.alt/1000

        drone_pos_msg = drone.get_message('GLOBAL_POSITION_INT')
        drone.lat = drone_pos_msg.lat / 1e7
        drone.lon = drone_pos_msg.lon / 1e7
        drone.heading = drone_pos_msg.hdg / 100
        drone.vx = drone_pos_msg.vx / 100
        drone.vy = drone_pos_msg.vy /100
        drone.speed = np.sqrt(drone.vx**2+drone.vy**2)
        drone.altitude = drone_pos_msg.alt/1000

        # Read redis stream for flight stage ("land", "follow")
        drone.stage =  rc.get_latest_stream_message("stage")[1]

        # Maybe be possible to set a desired_boat_speed to the same as drone speed and allow "straight down" landing
        # Exactly the same way current landing works, idk.

        # A catchup speed higher than impact speed will increase Gr ever so slightly because
        # The boat needs a little time to actually speed up and then distance to P3
        # Will be slightly less but with the same z_wanted so Gr has to increase a little
        catchup_speed = 3
        impact_speed = 2
        desired_boat_speed = drone.speed - impact_speed

        # Follow boat
        if drone.stage == "follow":

            # Calculate P2 and P1
            P2_distance = wp.calc_P2(drone.speed, desired_boat_speed, drone.altitude-boat.altitude+aim_under_boat, Gr)
            P2_lat, P2_lon = wp.calc_look_ahead_point(boat.deck_lat, boat.deck_lon, boat.heading-180, P2_distance)

            P1_distance = P2_distance + 20

            # Set boat's last recieved position as a waypoint for the drone
            target_lat, target_lon = wp.calc_look_ahead_point(P2_lat, P2_lon, boat.heading, 40)
            drone.follow_target([target_lat], [target_lon], [cruise_altitude])

            # Calculate the distance between drone and boat
            drone_distance_to_boat = wp.dist_between_coords(drone.lat, drone.lon, boat.deck_lat, boat.deck_lon)
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
            print(f"Drone altitude: {drone.altitude}")
            print("\n")
            boat_target_lat = boat.lat + 0.002
            boat_target_lon = boat.lon
            boat.set_guided_waypoint(boat_target_lat, boat_target_lon, 3)

            # Calculate where P2 is
            P2_distance = wp.calc_P2(drone.speed, desired_boat_speed, drone.altitude-boat.altitude+aim_under_boat, Gr)
            P2_lat, P2_lon = wp.calc_look_ahead_point(boat.deck_lat, boat.deck_lon, boat.heading-180, P2_distance) # -180 because behind

            # Calculate drone distance to boat and boat distance to P3 (target)
            drone_distance_to_boat = wp.dist_between_coords(drone.lat, drone.lon, boat.deck_lat, boat.deck_lon)
            boat_distance_to_target = wp.calc_landing_point_dist_boat(drone_distance_to_boat, drone.speed, desired_boat_speed)

            # Calculate where P3 (landing point) is
            P3_lat, P3_lon = wp.calc_look_ahead_point(boat.deck_lat, boat.deck_lon, boat.heading, boat_distance_to_target)

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
                boat_lh_lat, boat_lh_lon = wp.calc_look_ahead_point(boat.deck_lat, boat.deck_lon, boat.heading, 30)

                drone.follow_target([boat_lh_lat], [boat_lh_lon], [cruise_altitude])

            # When the drone passed P2 + lookahead it should start descending (landing)
            else:
                boat.set_speed(desired_boat_speed)
                # Calculate distance to previous iteration's P3, used along with prev_Gr to calculate z_wanted
                distance_to_prev_P3 = wp.dist_between_coords(drone.lat, drone.lon, prev_P3_lat, prev_P3_lon)

                # Calculate speed factor between relative and drone speed, 
                # needed to transform the lookahead from P2 (relative) frame to P3 (absolute) frame
                relative_speed = drone.speed - desired_boat_speed
                speed_factor = drone.speed/relative_speed

                # Calculate what altitude the drone should be at
                # Transform descent_lookahead to P3 frame
                # Add (boat_altitude - aim_under_boat) to account for not landing at 0m altitude
                # max() to no get negative z_wanted when the drone gets close to the boat
                z_wanted_lh = (min(cruise_altitude, (max(0, distance_to_prev_P3 - descent_lookahead*speed_factor))*prev_Gr + boat.altitude - aim_under_boat))
                z_wanted = (min(cruise_altitude, (max(0, distance_to_prev_P3))*prev_Gr + boat.altitude - aim_under_boat))

                # Fly towards P3 at boat's altitude
                print(f"P2 distance: {P2_distance}")
                print(f"Drone distance: {drone_distance_to_boat}")
                print(f"Boat to target: {boat_distance_to_target}")
                print(f"Dist to prev: {distance_to_prev_P3}")
                print("\n")

                # Calculate distance to new P3 and set prev_P3 to new P3
                distance_to_current_P3 = wp.dist_between_coords(drone.lat, drone.lon, P3_lat, P3_lon)
                prev_P3_lat, prev_P3_lon = P3_lat, P3_lon

                # Update prev_Gr
                # Same as before, transform descent_lookahead to P3 frame
                # Add (boat_altitude-aim_under_boat)/prev_Gr to transform the altitude offset to a longitudinal distance
                # Use z_wanted_lh because we need to look at where the drone is looking
                new_Gr = z_wanted_lh/(max(0, distance_to_current_P3 - descent_lookahead*speed_factor) + (boat.altitude - aim_under_boat)/prev_Gr)
                prev_Gr = new_Gr

                needed_Gr = (drone.altitude - boat.altitude + aim_under_boat)/(distance_to_current_P3)

                wanted_sink_rate = drone.speed*new_Gr

                # Calculate altitude error and a sink rate correction
                # Can use z_wanted_lh to calculate error if faster response is needed
                altitude_error = drone.altitude - z_wanted
                sink_rate_correction = altitude_error*altitude_error_gain
                needed_sink_rate = wanted_sink_rate + sink_rate_correction

                # Set sink rate of drone according to wanted sink rate and error correction
                drone.set_parameter("TECS_SINK_MIN", needed_sink_rate) # Default: 2.0
                drone.set_parameter("TECS_SINK_MAX", needed_sink_rate) # Default: 5.0

                print(f"z wanted: {z_wanted}")
                print(f"Drone speed: {drone.speed}")
                print(f"Gr: {new_Gr}")
                print(f"Needed Gr: {needed_Gr}")
                print(f"Altitude error: {altitude_error}")
                print(f"Wanted sink rate: {wanted_sink_rate}")
                print(f"Altitude: {drone.altitude}")
                print(f"Needed sink rate: {wanted_sink_rate + sink_rate_correction}")
                print("\n")

                started_descent = True

                drone_coordinates.append((drone.lat, drone.lon))
                # Save z_wanted not z_wanted_lh because we compare to where the drone currently should be
                z_wanteds.append(z_wanted)
                drone_altitudes.append(drone.altitude)
                needed_sink_rates.append(needed_sink_rate)
                wanted_sink_rates.append(wanted_sink_rate)

                if i >= 1:
                    dist_to_start_coord = wp.dist_between_coords(drone_coordinates[0][0], drone_coordinates[0][1],
                                                                drone_coordinates[i][0], drone_coordinates[i][1])
                    xs.append(dist_to_start_coord)

                # Update data dicts
                drone.data.update({"xs": xs,
                    "altitude": drone_altitudes,
                    "z_wanted": z_wanteds,
                    "needed_sr": needed_sink_rates,
                    "wanted_sr": wanted_sink_rates,
                    "Gr": Gr})
                
                boat.data.update({"altitude": boat.altitude})
                
                # Send data to redis stream
                rc.add_stream_message("drone data", drone.data)
                rc.add_stream_message("boat data", boat.data)
                rc.add_stream_message("needed_glide_ratio", needed_Gr)

                i += 1

                drone.follow_target([P3_lat, boat_target_lat], [P3_lon, boat_target_lon], 
                                    [boat.altitude-aim_under_boat, boat.altitude-aim_under_boat])

        elif drone.stage == "diversion":
            # Allow going into follow mode and starting a new descent
            started_descent = False
            # Implement actual diversion manuever and not just RTL for 3 sec
            drone.set_mode("RTL")
            sleep(3)
            # Go follow mode after diversion
            rc.add_stream_message("stage", "follow")
            # Set the Gr to the wanted Gr so the last needed_Gr is not saved
            rc.add_stream_message("needed_glide_ratio", Gr)
        
        elif drone.stage == "exit":
            # Shut down connection and gazebo            
            break
        
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









