from pymavlink import mavutil
import time

connection_string = 'udp:127.0.0.1:14550' 
master = mavutil.mavlink_connection(connection_string)

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received from system (system %u, component %u)" % (master.target_system, master.target_component))

# Function to set the mode
def set_mode(mode_name):
    mode_id = master.mode_mapping()[mode_name]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    print("Arming vehicle...")
    master.arducopter_arm()

    # Wait for acknowledgment that the vehicle is armed
    while True:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print("Vehicle armed successfully!")
                break
            else:
                print(f"Failed to arm vehicle. Result: {msg.result}")
                break
        time.sleep(1)

    print("Setting mode to GUIDED...")
    master.set_mode("GUIDED")

    # Wait for acknowledgment
    while True:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        if msg:
            print(f"Received acknowledgment: {msg}")
            if msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Mode set to GUIDED successfully!")
                    break
                else:
                    print(f"Failed to set mode. Result: {msg.result}")
                    break
        time.sleep(1)


# Arm the drone
def arm_vehicle():
    print("Arming vehicle...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    # Wait for the acknowledgment
    master.recv_match(type='COMMAND_ACK', blocking=True)
    print("Vehicle armed!")

# Take off to a target altitude
def takeoff(target_altitude):
    print(f"Taking off to {target_altitude} meters...")
    master.mav.command_long_send(
        target_system=1,
        target_component=1, 
        command=mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 
        confirmation=0,
        param1=0,
        param2=0,
        param3=0,
        param4=0,
        param5=0,
        param6=0,
        param7=target_altitude
    )
    # Wait for altitude to be reached
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        altitude = msg.relative_alt / 1000.0
        print(f"Current altitude: {altitude:.2f} meters")
        if altitude >= target_altitude * 0.95: 
            print("Target altitude reached!")
            break
        time.sleep(1)

# Go to a waypoint
def go_to_waypoint(latitude, longitude, altitude):
    print(f"Flying to waypoint: ({latitude}, {longitude}, {altitude})")
    master.mav.mission_item_send(
        master.target_system, master.target_component,
        0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  
        2,  
        1,  
        0, 0, 0, 0,  
        latitude, longitude, altitude
    )

def land():
    print(f"Landing...")
    master.mav.command_long_send(
        target_system=master.target_system,
        target_component=master.target_component,
        command=mavutil.mavlink.MAV_CMD_NAV_LAND,
        confirmation=0,
        param1=0, 
        param2=0, 
        param3=0, 
        param4=0, 
        param5=0,
        param6=0, 
        param7=0  
    )


# Main function
def main():
    set_mode("GUIDED")
    arm_vehicle()
    takeoff(5)
    go_to_waypoint(-35.3632, 149.1652, 5)
    time.sleep(6)
    set_mode("LAND")
    land()
    print("Mission complete!")

if __name__ == "__main__":
    main()
