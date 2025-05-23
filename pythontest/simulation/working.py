from pymavlink import mavutil
from subprocess import Popen, PIPE
from time import sleep
import time
import numpy as np
from redis_communication import RedisClient
import WP_calculation as wp

from Drone import Drone
from Boat import Boat
from Vehicle import Vehicle
import fluctuations as fl
from Guidance.kalman_OOSM import KalmanFilterXY

#TODO: 
# Add some fluctuations in boat movement, both lateral, (speed), and altitude - DONE
# And test drone's performance when both following and landing
# Test for different winds - DONE
# Test for changing winds (over time and for different altitudes) - DONE (Not altitude)
# Turbulence behind boat (possibly hard)
# Test how well drone holds altitude

# Add a way to know if the drone landed or not
# Test without aborting to see what will happen

def tester():
    # CONSTANTS
    R = 6371000 # Earth radius

    # FLAGS:
    started_descent = False # Bool to keep track when descent is started
    diversion_started = False # Check if diversion is started

    verbose = True
    use_filter = False
    fluct_boat_movement = False
    fluct_boat_alt = False
    fluct_drone_throttle = True
    maneuver_boat = False

    # PARAMETERS:
    Gr = 1/20 # Glide ratio
    needed_Gr = 1/20 # Glide ratio
    descent_lookahead = 4
    aim_under_boat = 0 # In meters, If we want the drone to aim slightly under the boat
    altitude_error_gain = 0.3
    speed_gain = 0.25
    diversion_distance = 40 # In meters, How far the drone should fly to the side when diverting
    impact_speed = 2
    P2_lookahead = 30

    # FLUCTUATIONS:
    boat_movement_fluctuation = 40 # Heading in degrees, 10 for landing, 30 for follow
    boat_alt_fluctuation = 6 # Meters
    throttle_fluct = 200
    turning_fluctuation_angle = 40 # Degrees

    # BASE VALUES:
    cruise_altitude = 15 # In meters
    boat_length = 2 # In meters, Eyeballed length from drone that is driving boat to rear deck of boat
    base_stall_speed = 12
    desired_boat_direction = 0
    desired_boat_altitude = 3
    base_throttle = 1800

    # Establish connection
    if verbose:
        print("Connecting to vehicles")
    drone_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    boat_connection = mavutil.mavlink_connection('udp:127.0.0.1:14560')

    if verbose:
        print("Waiting for heartbeat")
    drone_connection.wait_heartbeat()
    boat_connection.wait_heartbeat()
    if verbose:
        print("Heartbeat received")

    drone = Drone(drone_connection)
    boat = Boat(boat_connection)

    drone_msg = drone.get_message('HEARTBEAT')
    boat_msg = boat.get_message('HEARTBEAT')

    # Init redisclient
    rc = RedisClient()
    if verbose:
        print("Redis client initialized")
    # For missionhandler abort condition
    rc.add_stream_message("needed_glide_ratio", Gr)
    rc.add_stream_message("stage", "started")
    rc.send_message("stage", "started")

    
    drone_coordinates = []
    follow_diversion_data = {}
    P1_message = {}
    P2_message = {}
    P3_message = {}
    gr_message = {}

    start_vehicles_simulation(drone, boat, base_throttle)

    landing_iterator = 0
    dist_to_start_coord = 0
    iterator = 0

    # Main loop
    while True:

        timestamp = time.time()

        drone.update_possition_mavlink()

        drone_mission_msg = drone.get_message("MISSION_CURRENT")
        current_wp = drone_mission_msg.seq
        
        if iterator == 0 and use_filter:
            # kf = KalmanFilterXY(v = boat.speed, psi = boat.heading, init_lat = boat.deck_lat, init_lon = boat.deck_lon)
            kf = innit_filter(boat, boat_length)
            if verbose:
                print(f"Boat initial position: {boat.deck_lat}, {boat.deck_lon}")
                print(f"Boat initial speed: {boat.speed}")
                print(f"Boat initial heading: {boat.heading}")
                print(f"Boat initial lat: {boat.lat}")
                print(f"Boat initial lon: {boat.lon}")
                print(f"Boat initial deck lat: {boat.deck_lat}")
                print(f"Boat initial deck lon: {boat.deck_lon}")


        # Simulateing sparse updates
        if not use_filter:
            update_without_kf(boat, boat_length)
            # Update boat position without filter
        elif iterator%3 == 0:
            update_boat_position(kf, boat, boat_length, timestamp)
            # Update boat position 
        else:
            predict_kf(kf, boat, boat_length, timestamp)

        # Save kf state in CSV
        if use_filter:
            boat.data.update({"time": timestamp,
                            "kf_x": kf.x[0][0],
                            "kf_y": kf.x[1][0],
                            "kf_lat": kf.lat,
                            "kf_lon": kf.lon,
                            "kf_speed": kf.x[2][0],
                            "kf_heading": kf.x[3][0],
                            "real_heading": boat.heading,
                            "real_lat": boat.lat_sim,
                            "real_lon": boat.lon_sim})
        
        if verbose and use_filter:
             print(f"kf x: {kf.x[0][0]}, kf y: {kf.x[1][0]}")

        drone_wind_msg = drone.get_message('WIND')
        wind_speed = drone_wind_msg.speed
        wind_direction = drone_wind_msg.direction # 0 is wind blowing negative latitude (south?)

        print(f"Wind Speed: {wind_speed}")
        print(f"Wind Direction: {wind_direction}")
        print(f"Drone heading: {drone.heading}")

        # Read redis stream for flight stage ("land", "follow")
        drone.stage =  rc.get_latest_stream_message("stage")[1]

        # Maybe be possible to set a desired_boat_speed to the same as drone speed and allow "straight down" landing
        # Exactly the same way current landing works, idk.


        wind_stall_speed = wind_speed*np.cos(np.deg2rad(drone.heading - wind_direction))
        drone_total_stall_speed = base_stall_speed - wind_stall_speed

        print("\n")
        print(f"Wind correction stall speed: {wind_stall_speed}")
        print(f"Total stall speed: {drone_total_stall_speed}")
        print("\n")

        # A catchup speed higher than impact speed will increase Gr ever so slightly because
        # The boat needs a little time to actually speed up and then distance to P3
        # Will be slightly less but with the same z_wanted so Gr has to increase a little

        desired_boat_speed = drone.speed - impact_speed

        # Fluctuate values
        # Random noise to desired heading
        # commanded_boat_direction = fl.boat_movement(fluct_boat_movement, 
        #                                   boat_movement_fluctuation, desired_boat_direction)

        # Random walk for heading
        commanded_boat_direction = fl.boat_movement(fluct_boat_movement, 
                                    boat_movement_fluctuation, boat.heading)
        
        if maneuver_boat:
            if iterator%12 == 0:
                commanded_boat_direction = boat.heading + turning_fluctuation_angle
        
        commanded_boat_altitude = fl.boat_altitude(fluct_boat_alt, boat_alt_fluctuation, 
                                         desired_boat_altitude, iterator)
        
        commanded_drone_throttle = fl.drone_throttle(fluct_drone_throttle, throttle_fluct, base_throttle)
        drone.set_servo(3, commanded_drone_throttle)
        
        # Calculate the distance between drone and boat
        drone_distance_to_boat = wp.dist_between_coords(drone.lat, drone.lon, boat.deck_lat, boat.deck_lon)

        P2_altitude = cruise_altitude - boat.altitude + aim_under_boat
        P2_distance = wp.calc_P2(drone.speed, desired_boat_speed, P2_altitude, Gr)
        P2_lat, P2_lon = wp.calc_look_ahead_point(boat.deck_lat, boat.deck_lon, boat.heading-180, P2_distance) # -180 because behind
        # P2_distance_to_drone = np.abs(wp.dist_between_coords(drone.lat, drone.lon, P2_lat, P2_lon))
        P2_distance_to_drone = (drone_distance_to_boat - P2_distance)

        P2_message.update({"time": timestamp,
                          "lat": P2_lat,
                          "lon": P2_lon,
                          "alt": cruise_altitude,
                          "distance": P2_distance_to_drone})
        rc.send_message('P2', P2_message)

        P1_lat, P1_lon = wp.calc_look_ahead_point(P2_lat, P2_lon, boat.heading-180, 20)
        P1_distance = wp.dist_between_coords(P1_lat, P1_lon, boat.deck_lat, boat.deck_lon)
        # P1_distance_to_drone = np.abs(wp.dist_between_coords(drone.lat, drone.lon, P1_lat, P1_lon))
        P1_distance_to_drone = (drone_distance_to_boat - P1_distance)
        P1_message.update({"time": timestamp,
                           "lat": P1_lat,
                           "lon": P1_lon,
                           "alt": cruise_altitude,
                           "distance": P1_distance_to_drone})
        rc.send_message('P1', P1_message)

        follow_diversion_data.update({"time": timestamp,
                                        "P2_distance": P2_distance,
                                        "stall_speed": drone_total_stall_speed,
                                        "boat_speed": boat.speed,
                                        "drone_distance": drone_distance_to_boat})
                

        rc.add_stream_message("follow diversion", follow_diversion_data)
        rc.send_message("follow diversion", follow_diversion_data)

        boat_distance_to_target = wp.calc_landing_point_dist_boat(drone_distance_to_boat, drone.speed, desired_boat_speed)

        # Calculate where P3 (landing point) is
        P3_lat, P3_lon = wp.calc_look_ahead_point(boat.deck_lat, boat.deck_lon, boat.heading, boat_distance_to_target)
        distance_to_current_P3 = wp.dist_between_coords(drone.lat, drone.lon, P3_lat, P3_lon)
        
        P3_message.update({"time": timestamp,
                           "lat": P3_lat,
                           "lon": P3_lon,
                           "alt": boat.altitude,
                           "distance": distance_to_current_P3})
        rc.send_message('P3', P3_message)

        

        # Point before P3 to help the drone line up behind the boat better
        # Especially when the boat is maneuvering
        intermediate_dist = distance_to_current_P3*2/3
        intermediate_P3_lat, intermediate_P3_lon = wp.calc_look_ahead_point(P3_lat, P3_lon, boat.heading-180, intermediate_dist)

        iterator += 1

        # Follow boat
        if drone.stage == "follow":

            # Move boat
            boat_target_lat, boat_target_lon = wp.calc_look_ahead_point(boat.lat, boat.lon, commanded_boat_direction, 200) 
            boat.set_guided_waypoint(boat_target_lat, boat_target_lon, commanded_boat_altitude)

            # Set boat's last recieved position as a waypoint for the drone
            target_lat, target_lon = wp.calc_look_ahead_point(P2_lat, P2_lon, boat.heading, P2_lookahead)
            drone.follow_target([target_lat], [target_lon], [cruise_altitude])

            # Dont just calculate dist between drone and P1 because it will still be 
            # Positive if drone is in front of P1
            dist_behind_P1 = drone_distance_to_boat - P1_distance
            wanted_boat_speed = drone.speed - dist_behind_P1*speed_gain
            boat.set_speed(max(drone_total_stall_speed, wanted_boat_speed)) # Drone stall speed is also affected by wind

            print(f"P1 distance_ {P1_distance}")
            print(f"Drone distance: {drone_distance_to_boat}")
            print(f"Drone to P1: {P1_distance_to_drone}")
            print(f"Desired boat speed: {max(drone_total_stall_speed, wanted_boat_speed)}")
            print(f"Actual boat speed: {boat.speed}")
            print(f"Relative speed: {drone.speed - boat.speed}")

        # Land on boat
        elif drone.stage == "land":

            # Move boat
            boat_target_lat, boat_target_lon = wp.calc_look_ahead_point(boat.lat, boat.lon, commanded_boat_direction, 200) 
            boat.set_guided_waypoint(boat_target_lat, boat_target_lon, commanded_boat_altitude)

            print(f"Drone altitude: {drone.altitude}")
            print("\n")

            # If drone is behind P2 + lookahead it should just keep flying towards the boat at cruise_altitude
            if (drone_distance_to_boat > P2_distance + descent_lookahead) and (not started_descent):
                boat.set_speed(desired_boat_speed)

                print(f"P2 distance: {P2_distance}")
                print(f"Drone distance: {drone_distance_to_boat}")
                print(f"Boat to target: {boat_distance_to_target}")
                print("\n")
                # Update prev_P3 and prev_Gr until we actually start landing
                prev_P3_lat, prev_P3_lon = P3_lat, P3_lon
                prev_Gr = Gr
                                
                # Follow boat with some lookahead (30m) to smooth out the path
                boat_lh_lat, boat_lh_lon = wp.calc_look_ahead_point(boat.deck_lat, boat.deck_lon, boat.heading, 30)

                drone.follow_target([boat_lh_lat, boat_target_lat], 
                                    [boat_lh_lon, boat_target_lon], 
                                    [cruise_altitude, cruise_altitude])

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
                print(f"Previous Gr: {prev_Gr}")
                print("\n")

                # Calculate distance to new P3 and set prev_P3 to new P3
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
                if landing_iterator >= 1:
                    dist_to_start_coord = wp.dist_between_coords(drone_coordinates[0][0], drone_coordinates[0][1],
                                                                drone_coordinates[landing_iterator][0], drone_coordinates[landing_iterator][1])

                # Update data dicts
                drone.data.update({"time": timestamp,
                    "xs": dist_to_start_coord,
                    "altitude": drone.altitude,
                    "z_wanted": z_wanted,
                    "needed_sr": needed_sink_rate,
                    "wanted_sr": wanted_sink_rate,
                    "actual_sr": drone.vz,
                    "Gr": Gr})
                
                boat.data.update({"time": timestamp,
                                  "altitude": boat.altitude,
                                  "lat": boat.lat,
                                  "lon": boat.lon})
                
                # Send data to redis stream
                rc.send_message("drone data", drone.data)
                rc.send_message("boat data", boat.data)
                gr_message.update({"time": timestamp,
                                   "Gr": Gr,
                                   "needed_Gr": needed_Gr})
                rc.send_message("glide ratio", gr_message)
                rc.add_stream_message("needed_glide_ratio", needed_Gr)

                # Drone has landed on boat
                if drone.altitude < boat.altitude + 1.5:
                    rc.send_message("stage", "diversion")
                    rc.add_stream_message("stage", "diversion")

                landing_iterator += 1

                drone.follow_target([intermediate_P3_lat, P3_lat, boat_target_lat], 
                                    [intermediate_P3_lon, P3_lon, boat_target_lon], 
                                    [boat.altitude-aim_under_boat, boat.altitude-aim_under_boat, boat.altitude-aim_under_boat])

        elif drone.stage == "diversion":
            print("DIVERTING", flush=True)

            boat_target_lat, boat_target_lon = wp.calc_look_ahead_point(boat.lat, boat.lon, commanded_boat_direction, 200) 
            boat.set_guided_waypoint(boat_target_lat, boat_target_lon, commanded_boat_altitude)

            # Allow going into follow mode and starting a new descent
            started_descent = False

            # Can be made into a method in Drone class
            if not diversion_started:
                diversion_lat, diversion_lon = wp.calc_look_ahead_point(drone.lat, drone.lon, drone.heading+110, diversion_distance)
                return_lat, return_lon = wp.calc_look_ahead_point(drone.lat, drone.lon, drone.heading, 20)

                drone.follow_target([diversion_lat, return_lat],
                                    [diversion_lon, return_lon],
                                    [cruise_altitude, cruise_altitude])
                diversion_started = True
            
            dist_to_return_point = wp.dist_between_coords(drone.lat, drone.lon, 
                                                             return_lat, return_lon)
        
            if dist_to_return_point < 40 and current_wp == 2:
                # Go follow mode after diversion
                rc.send_message("stage", "follow")
                rc.add_stream_message("stage", "follow")
                diversion_started = False
                # Set the Gr to the wanted Gr so the last needed_Gr is not saved
                rc.add_stream_message("needed_glide_ratio", Gr)

            # Send data to redis stream for plotting
            gr_message.update({"time": timestamp,
                                "Gr": Gr,
                                "needed_Gr": needed_Gr})
            rc.send_message("glide ratio", gr_message)
        
        elif drone.stage == "exit":
            # Shut down connection and gazebo
            print("Shutting down connection and gazebo")        
            break
        
        # No command
        else:
            boat.set_speed(desired_boat_speed)
            # Move boat
            boat_target_lat, boat_target_lon = wp.calc_look_ahead_point(boat.lat, boat.lon, commanded_boat_direction, 200) 
            boat.set_guided_waypoint(boat_target_lat, boat_target_lon, commanded_boat_altitude)

            # Loiter until command recieved
            drone.set_mode("GUIDED")
        
        

        # add stuff to redis stream if they have not been populated this itteration
        print('here')
        if "time" not in drone.data or ("time" in drone.data and drone.data["time"] != timestamp):
            print("updating drone data")
            drone.data.update({"time": timestamp,
                            "xs": 0,
                            "altitude": drone.altitude,
                            "z_wanted": cruise_altitude,
                            "needed_sr": 0,
                            "wanted_sr": 0,
                            "actual_sr": drone.vz,
                            "Gr": Gr})
            
            # Send data to redis stream
            rc.send_message("drone data", drone.data)
        
        # Flush all messages
        boat.flush_messages()
        drone.flush_messages()


def innit_filter(boat: Boat, boat_length):
    # Get initial position of boat
    boat.update_possition_mavlink()
    boat.deck_lat, boat.deck_lon = wp.calc_look_ahead_point(boat.lat, boat.lon, boat.heading-180, boat_length)

    # Set initial positional values
    boat.inital_lat = boat.lat
    boat.inital_lon = boat.lon
    boat.x = 0
    boat.y = 0

    #innitiate kf filter
    kf = KalmanFilterXY(u = boat.vx, v = boat.vy, heading= boat.heading, init_lat = boat.lat, init_lon = boat.lon)

    return kf

def update_without_kf(boat: Boat, boat_length):
    boat.update_possition_mavlink()
    boat.deck_lat, boat.deck_lon = wp.calc_look_ahead_point(boat.lat, boat.lon, boat.heading-180, boat_length)


def update_boat_position(kf: KalmanFilterXY, boat: Boat, boat_length, timestamp):
    boat.update_possition_mavlink()

    # v = boat.speed * np.cos(np.deg2rad(boat.heading))
    # u = boat.speed * np.sin(np.deg2rad(boat.heading))
    # z = np.array([[boat.lat], [boat.lon], [boat.heading], [u], [v]])

    z = np.array([[boat.lat], [boat.lon], [boat.heading], [boat.vx], [boat.vy]])

    #update filter with new position
    kf.update_w_latlon(z, timestamp)

    boat.sim_lat = boat.lat
    boat.sim_lon = boat.lon

    update_object(boat, kf, boat_length)


def predict_kf(kf, boat, boat_length, timestamp):
    kf.predict(timestamp)

    update_object(boat, kf, boat_length)

    
def update_object(obj: Vehicle, filter: KalmanFilterXY, boat_length = 0):
    # Update the object's position using the Kalman filter
    obj.lat = filter.lat
    obj.lon = filter.lon
    # obj.speed = np.sqrt((filter.x[3][0])**2 + (filter.x[4][0])**2)
    obj.speed = filter.x[3][0]
    obj.heading = filter.x[2][0]

    obj.x = filter.x[0][0]
    obj.xdot = filter.x[3][0]
    obj.y = filter.x[1][0]
    obj.ydot = filter.x[4][0]
    obj.psi = filter.x[2][0]
    obj.psidot = filter.x[5][0]

    # Calculate the deck position
    obj.deck_lat, obj.deck_lon = wp.calc_look_ahead_point(obj.lat, obj.lon, obj.heading-180, boat_length)


def start_vehicles_simulation(drone: Drone, boat: Boat, base_throttle):
    # Parameter settings
    drone.set_parameter("TECS_SPDWEIGHT", 0.0) # Default: -1
    drone.set_parameter("TECS_TIME_CONST", 3.0) # Default: 5.0
    drone.set_parameter("TECS_THR_DAMP", 0.5) # Default: 0.5
    drone.set_parameter("TECS_PITCH_MIN", 0.0) # Default: 0.0
    drone.set_parameter("TECS_PTCH_DAMP", 0.3) # Default: 0.3
    drone.set_parameter("TECS_HGT_OMEGA", 3.0) # Default: 3.0
    drone.set_parameter("TECS_VERT_ACC", 8) # Default: 7
    drone.set_parameter("TECS_INTEG_GAIN", 0.5) # Default: 0.3
    drone.set_parameter("TECS_LAND_TCONST", 1.0) # Default 2.0
    drone.set_parameter("WP_RADIUS", 5) # Default 90

    sleep(2)

    print("Setting modes")
    # Set modes
    drone.set_mode("FBWB")
    boat.set_mode("GUIDED")

    print("Arming vehicles")

    # Arm vehicles
    drone.arm_vehicle()
    sleep(2)
    boat.arm_vehicle()
    sleep(2)

    print("Taking off vehicles")

    # Takeoff vehicles
    boat.takeoff(3)
    sleep(6)
    drone.set_servo(3, base_throttle)
    sleep(1)

    print("Takeoff complete")