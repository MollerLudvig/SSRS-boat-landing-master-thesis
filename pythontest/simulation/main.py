from pymavlink import mavutil
import asyncio

import tester
import working

from Drone import Drone
from Boat import Boat
from Guidance.kalman_OOSM import KalmanFilterXY

async def starter(beacon = False, AIS = False, camera = False, simulation = True):
    
    if not simulation:
        kf = innit_filter(beacon, AIS, camera, simulation)

    # Establish connection
    drone_connection = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    boat_connection = mavutil.mavlink_connection('udp:127.0.0.1:14560')

    # Wait for heartbeat
    drone_connection.wait_heartbeat()
    boat_connection.wait_heartbeat()

    # Create instances of Drone and Boat
    drone = Drone(drone_connection)
    boat = Boat(boat_connection)

    # Create tasks for getting data and running main loop
    droneTask = asyncio.create_task(drone.gat_global_position())
    boatTask = asyncio.create_task(boat.gat_global_position())
    testerTask = asyncio.create_task(working.tester(drone, boat, simulation))

    # Run concurrently forever (or until one fails/stops)
    await asyncio.gather(droneTask, boatTask, testerTask)


def innit_filter(beacon, AIS, camera, simulation):
    # Initialize the Kalman filter with the appropriate parameters
    #kf = KalmanFilter(



    return False


if __name__ == '__main__':
    asyncio.run(starter())

