from pymavlink import mavutil
from subprocess import Popen, PIPE
from time import sleep

from Drone import Drone

# Start the Gazebo simulation
gazebo_sim = "gz sim island.sdf -r".split()
Popen(gazebo_sim)
sleep(1)

# Start the ArduPlane simulation
command_list_drone = "sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --map --console --out 0.0.0.0:14551 --out 0.0.0.0:14552 -i0 -L Kattegatt".split()
drone_process = Popen(command_list_drone, stdin=PIPE)
sleep(1)

# Connect to the drone
drone_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
drone_connection.wait_heartbeat()
print("Heartbeat received!")

drone = Drone(drone_connection)

def set_parameter(param_name, value):
    """Set a parameter on the flight controller."""
    drone_connection.mav.param_set_send(
        drone_connection.target_system,
        drone_connection.target_component,
        param_name.encode(),  # Corrected: Encode directly
        float(value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    print(f"Setting {param_name} to {value}")

    # Wait for confirmation
    while True:
        message = drone_connection.recv_match(type='PARAM_VALUE', blocking=True)
        if message and message.param_id == param_name:  # Fixed param_id comparison
            print(f"{param_name} updated to {message.param_value}")
            break


def tester():
    # Ensure connection is ready
    drone_connection.wait_heartbeat()
    print("Connected to the drone!")

    # **Set SERVO3_FUNCTION for throttle control**
    set_parameter("SERVO3_FUNCTION", 70)
    sleep(2)

    # Set mode to MANUAL first, then AUTO for takeoff
    drone.arm_vehicle()
    sleep(2)
    drone.set_mode("TAKEOFF")
    sleep(2)

    # Wait for takeoff
    sleep(10)

    # Switch to RTL mode for landing
    drone.set_mode("RTL")
    print("Returning to launch...")

    # Wait for landing
    sleep(30)

    print("Landed successfully!")

# Run the test function
tester()

