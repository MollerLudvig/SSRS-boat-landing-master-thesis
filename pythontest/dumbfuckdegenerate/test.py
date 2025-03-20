
def tester():
    from pymavlink import mavutil
    from subprocess import Popen, PIPE
    from time import sleep
    import numpy as np
    import math

    # Function for setting flight mode
    def set_parameter(connection, param_name, param_value):
        connection.mav.param_set_send(
            connection.target_system, connection.target_component,
            param_name.encode(), float(param_value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def set_mode(drone_connection, mode):
        mode_id = drone_connection.mode_mapping().get(mode) 

        while True:
            drone_connection.mav.set_mode_send(
            drone_connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
            msg = drone_connection.recv_match(type='HEARTBEAT', blocking=True)
            if msg and msg.custom_mode == mode_id:
                print(f"Mode set to {mode}")
                return True
            sleep(1)

    def set_guided_waypoint(connection, lat, lon, alt):
        # Send a new target position in GUIDED mode
        bmsg1 = connection.mav.mission_item_int_send(
            connection.target_system,  # Target system ID (0 for broadcast)
            connection.target_component,  # Target component ID (0 for broadcast)
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
        #connection.mav.send(bmsg1)
        print(f"New waypoint set: lat={lat}, lon={lon}, alt={alt}")

    def set_speed(connection, speed):
        """Set the vehicle's target speed."""
        speed_msg = connection.mav.command_long_encode(
            connection.target_system,  # Target system ID
            connection.target_component,  # Target component ID
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  # Command ID for changing speed
            0,  # Confirmation
            0,  # Speed type: 0 = airspeed, 1 = ground speed
            speed,  # Target speed (m/s)
            -1,  # Throttle (ignored)
            0, 0, 0, 0  # Unused parameters
        )
        connection.mav.send(speed_msg)
        print(f"Speed set to {speed} m/s")

    def flush_messages(connection):
        while True:
            msg = connection.recv_msg()
            if msg is None:
                break

    def upload_mission(connection, waypoints):
        # Clear existing mission
        connection.mav.mission_clear_all_send(connection.target_system, connection.target_component)

        # Send mission count
        connection.mav.mission_count_send(connection.target_system, connection.target_component, len(waypoints))

        for i, (lat, lon, alt) in enumerate(waypoints):
            msg = mavutil.mavlink.MAVLink_mission_item_int_message(
                target_system=connection.target_system,
                target_component=connection.target_component,
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
            connection.mav.send(msg)
        
        print(f"Uploaded {len(waypoints)} waypoints.")

    def follow_boat(connection, lat, lon, alt):

        waypoint1 = (lat-0.0005, lon, alt)
        waypoint2 = (lat, lon, alt)

        print(f"Uploading mission with 3 waypoints: , {waypoint2}, ")
        upload_mission(connection, [waypoint1,waypoint2])

        set_mode(connection, "FBWA")
        set_mode(connection, "AUTO")
        print("Drone is following the boat")
        # Save roll value from the AUTO mode and set that same value
        # For the short moment it is in FBWB mode to keep turning



    # Setup simulation enviroment and connections
    command_list_drone = "sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --map --console --out 0.0.0.0:14551 --out 0.0.0.0:14552 -i0 -L Kattegatt".split()
    command_list_boat = "sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out 0.0.0.0:14561 --out 0.0.0.0:14562 -i1 -L Kattegatt".split()
    gazebo_sim = "gz sim island.sdf -r".split()

    Popen(gazebo_sim)
    sleep(1)
    drone_process = Popen(command_list_drone, stdin=PIPE)
    sleep(1)
    boat_process = Popen(command_list_boat, stdin=PIPE)

    drone_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    boat_connection = mavutil.mavlink_connection('udp:127.0.0.1:14560')

    drone_connection.wait_heartbeat()
    boat_connection.wait_heartbeat()

    drone_msg = drone_connection.recv_match(type='HEARTBEAT', blocking=True)
    boat_msg = boat_connection.recv_match(type="HEARTBEAT", blocking=True)

    sleep(5)
    set_parameter(drone_connection, "SERVO3_FUNCTION", 70)

    set_mode(boat_connection, "GUIDED")

    # Continuously try to arm drone
    while not drone_connection.motors_armed():
        drone_connection.mav.command_long_send(
            drone_connection.target_system, drone_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        )

        drone_msg = drone_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)

        print("trying to arm drone")
        sleep(1)
    print("drone is armed")

    sleep(5)

    # Continuously try to arm boat
    while not boat_connection.motors_armed():
        boat_connection.mav.command_long_send(
            boat_connection.target_system, boat_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0
        )

        boat_msg = boat_connection.recv_match(type='HEARTBEAT', blocking=True, timeout=5)

        print("trying to arm boat")
        sleep(1)
    print("boat is armed")

    sleep(2)

    boat_connection.mav.command_long_send(
        boat_connection.target_system, boat_connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 3
    )

    sleep(10)
    
    boat_connection.mav.request_data_stream_send(
        boat_connection.target_system,
        boat_connection.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        5,  # Increased frequency (2 Hz)
        1
    )
    
    set_mode(drone_connection, "TAKEOFF")
    sleep(10)

    i = 0
    while True:
        print("In while loop")
        current_msg = boat_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        lat = current_msg.lat / 1e7
        lon = current_msg.lon / 1e7

        boat_target_lat = lat + 0.002
        boat_target_lon = lon
        set_speed(boat_connection, 13)
        set_guided_waypoint(boat_connection, boat_target_lat, boat_target_lon, 3)
        flush_messages(boat_connection)

        if i > 9:
            
            follow_boat(drone_connection, lat, lon, 10)

            
        i += 1
        sleep(1)


