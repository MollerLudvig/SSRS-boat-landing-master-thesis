import matplotlib.pyplot as plt
# from pymavlink import mavutil
import time
import numpy as np
from VehicleMonitor import VehicleMonitor
from variabels import VehicleData  # Import the VehicleData class
from redisCallbacks import RedisCallbacks
from redis_communication import RedisClient
from threading import Thread, Lock
from coordinate_conv import latlon_to_ned, latlon_to_xy
from dataclasses import dataclass, field
from collission import ColissionData, is_landed


Simulation = True

# Configure window display options
enableGlobalWindow = True
enableDroneAttitudeWindow = False
enableBoatAttitudeWindow = False
enableDroneVelocityWindow = False
enableBoatVelocityWindow = False
enableRelativeVelocityWindow = True
enableWindWindow = False
enableCollisionWindow = False
enableDistToP = True
savePlots = False

displayed_indices = 100

@dataclass
class Colors:
    boat : str = 'green'
    drone : str = 'blue'
    p1 : str = 'purple'
    p2 : str = 'orange'
    p3 : str = 'red'
    collision : str = 'red'
colors = Colors()


redis_client = RedisClient(host="localhost", port=6379)
start_time = time.time()
callbacks = RedisCallbacks(redis_client, start_time=start_time)
print("Redis client initialized.")
callbacks.start_listening()
print("Redis callbacks started.")


# Initialize vehicles  
drone = VehicleMonitor(name="Drone", udpPort=14551, color='blue', start_time=start_time)
boat = VehicleMonitor(name="Boat", udpPort=14561, color='green', start_time=start_time)
collision_data = ColissionData()

drone.connect()
boat.connect()

time.sleep(5)

# Initialize data containers with the VehicleData class
droneData = VehicleData(drone)
boatData = VehicleData(boat)

# Create figures and axes based on configuration
if enableGlobalWindow:
    figGlobal, axGlobal = plt.subplots(1, 1, figsize=(8, 6), num="Global Position View")
    figGlobal.suptitle("Global Position (Lat/Lon)", fontsize=14)

if enableDroneAttitudeWindow:
    figDroneAtt, axsDroneAtt = plt.subplots(3, 1, figsize=(10, 8), num="Drone Attitude")
    figDroneAtt.suptitle(f"Drone Attitude & Sensor Data", fontsize=14)

if enableBoatAttitudeWindow:
    figBoatAtt, axsBoatAtt = plt.subplots(3, 1, figsize=(10, 8), num="Boat Attitude")
    figBoatAtt.suptitle(f"Boat Attitude & Sensor Data", fontsize=14)

if enableDroneVelocityWindow:
    figDroneVel, axsDroneVel = plt.subplots(3, 1, figsize=(10, 8), num="Drone Velocity")
    figDroneVel.suptitle(f"Drone Velocity Components", fontsize=14)

if enableBoatVelocityWindow:
    figBoatVel, axsBoatVel = plt.subplots(3, 1, figsize=(10, 8), num="Boat Velocity")
    figBoatVel.suptitle(f"Boat Velocity Components", fontsize=14)

if enableRelativeVelocityWindow:
    figRelVel, axsRelVel = plt.subplots(4, 1, figsize=(10, 10), num="Relative Velocity", constrained_layout=True)
    figRelVel.suptitle(f"Drone-Boat Relative Velocity", fontsize=14)

if enableWindWindow:
    figWind, axsWind = plt.subplots(2, 1, figsize=(8, 8), num="Wind Data")
    figWind.suptitle(f"Wind Speed and Direction", fontsize=14)

if enableCollisionWindow:
    figCollision, axsCollision = plt.subplots(3, 2, figsize=(8, 6), num="Collision Detection")
    figCollision.suptitle(f"Collision Detection", fontsize=14)

if enableDistToP:
    figDistToP, axsDistToP = plt.subplots(3, 1, figsize=(8, 6), num="Distance to P1, P2, P3")
    figDistToP.suptitle(f"Distance to P1, P2, P3", fontsize=14)



def get_vector_magnitude(x, y, z=0):
    return np.sqrt(x**2 + y**2 + z**2)

def update_plot(_):
    """Update all plot windows with latest data"""

    # Update data containers (thread-safe)
    droneData.update()
    boatData.update()

    # Check if landed ( or maby rather distance to get collision data)
    if enableCollisionWindow:
        is_landed(collision_data, boatData, droneData, threshold = 3.0, 
                  offset_transform = [-2.5, 0.0, 0.0], max_time_delta = 0.065)

    # 1. Update Global Position Window
    if enableGlobalWindow:
        axGlobal.clear()
        
        # Prioritize simulation data when available
        if callbacks.P1:
            axGlobal.plot(callbacks.P1.lon[-5:], callbacks.P1.lat[-5:], 
                         'o-', markersize=10, label="P1 (SIM)", color=colors.p1, )

        if boatData.simulation.lat:
            axGlobal.plot(boatData.simulation.lon[-10:], boatData.simulation.lat[-10:], 
                         'o-', markersize=10, label="Boat (SIM)", color=colors.boat, fillstyle='none')
        elif boatData.gps.lat:
            print('Simdata not available, using GPS data (BOAT)')
            axGlobal.plot(boatData.gps.lon[-8:], boatData.gps.lat[-8:], 
                         'o-', markersize=10, label="Boat (GPS)", color=colors.boat, alpha=0.7, fillstyle='none')

        if droneData.simulation.lat:
            axGlobal.plot(droneData.simulation.lon[-10:], droneData.simulation.lat[-10:], 
                         'x-', markersize=10, label="Drone (SIM)", color=colors.drone)
        elif droneData.gps.lat:
            print('Simdata not available, using GPS data (DRONE)')
            axGlobal.plot(droneData.gps.lon[-8:], droneData.gps.lat[-8:], 
                         'x-', markersize=10, label="Drone (GPS)", color=colors.drone, alpha=0.7)
            
        axGlobal.set_xlabel("Longitude")
        axGlobal.set_ylabel("Latitude") 
        axGlobal.legend()
        axGlobal.grid(True)
        if any([droneData.gps.lat, droneData.simulation.lat, 
                boatData.gps.lat, boatData.simulation.lat]):
            axGlobal.axis('equal')
            
    # 2. Update Drone Attitude Window
    if enableDroneAttitudeWindow and len(axsDroneAtt) == 3:
        # Roll, Pitch, Yaw plot
        axsDroneAtt[0].clear()
        if droneData.simulation.roll:
            axsDroneAtt[0].plot(droneData.simulation.time[-displayed_indices:], droneData.simulation.roll[-displayed_indices:], 
                               label='Roll (SIM)', color='red')
            axsDroneAtt[0].plot(droneData.simulation.time[-displayed_indices:], droneData.simulation.pitch[-displayed_indices:], 
                               label='Pitch (SIM)', color='green')
            axsDroneAtt[0].plot(droneData.simulation.time[-displayed_indices:], droneData.simulation.yaw[-displayed_indices:], 
                               label='Yaw (SIM)', color='blue')
        elif droneData.attitude.roll:
            axsDroneAtt[0].plot(droneData.attitude.time[-displayed_indices:], droneData.attitude.roll[-displayed_indices:], 
                               label='Roll', color='red', alpha=0.7)
            axsDroneAtt[0].plot(droneData.attitude.time[-displayed_indices:], droneData.attitude.pitch[-displayed_indices:], 
                               label='Pitch', color='green', alpha=0.7)
            axsDroneAtt[0].plot(droneData.attitude.time[-displayed_indices:], droneData.attitude.yaw[-displayed_indices:], 
                               label='Yaw', color='blue', alpha=0.7)
        axsDroneAtt[0].set_ylabel("Degrees")
        axsDroneAtt[0].set_title("Attitude")
        axsDroneAtt[0].legend()
        axsDroneAtt[0].grid(True)
        
        # Angular rates plot
        axsDroneAtt[1].clear()
        if droneData.simulation.xgyro:
            axsDroneAtt[1].plot(droneData.simulation.time[-displayed_indices:], droneData.simulation.xgyro[-displayed_indices:], 
                               label='Roll rate (SIM)', color='red')
            axsDroneAtt[1].plot(droneData.simulation.time[-displayed_indices:], droneData.simulation.ygyro[-displayed_indices:], 
                               label='Pitch rate (SIM)', color='green')
            axsDroneAtt[1].plot(droneData.simulation.time[-displayed_indices:], droneData.simulation.zgyro[-displayed_indices:], 
                               label='Yaw rate (SIM)', color='blue')
        elif droneData.attitude.roll_speed:
            axsDroneAtt[1].plot(droneData.attitude.time[-displayed_indices:], droneData.attitude.roll_speed[-displayed_indices:], 
                               label='Roll rate', color='red', alpha=0.7)
            axsDroneAtt[1].plot(droneData.attitude.time[-displayed_indices:], droneData.attitude.pitch_speed[-displayed_indices:], 
                               label='Pitch rate', color='green', alpha=0.7)
            axsDroneAtt[1].plot(droneData.attitude.time[-displayed_indices:], droneData.attitude.yaw_speed[-displayed_indices:], 
                               label='Yaw rate', color='blue', alpha=0.7)
        axsDroneAtt[1].set_ylabel("Degrees/sec")
        axsDroneAtt[1].set_title("Angular Rates")
        axsDroneAtt[1].legend()
        axsDroneAtt[1].grid(True)
        
        # Accelerations plot
        axsDroneAtt[2].clear()
        if droneData.simulation.xacc:
            axsDroneAtt[2].plot(droneData.simulation.time[-displayed_indices:], droneData.simulation.xacc[-displayed_indices:], 
                               label='X Accel (SIM)', color='red')
            axsDroneAtt[2].plot(droneData.simulation.time[-displayed_indices:], droneData.simulation.yacc[-displayed_indices:], 
                               label='Y Accel (SIM)', color='green')
            axsDroneAtt[2].plot(droneData.simulation.time[-displayed_indices:], droneData.simulation.zacc[-displayed_indices:], 
                               label='Z Accel (SIM)', color='blue')
        elif droneData.imu.ax:
            axsDroneAtt[2].plot(droneData.imu.time[-displayed_indices:], droneData.imu.ax[-displayed_indices:], 
                               label='X Accel', color='red', alpha=0.7)
            axsDroneAtt[2].plot(droneData.imu.time[-displayed_indices:], droneData.imu.ay[-displayed_indices:], 
                               label='Y Accel', color='green', alpha=0.7)
            axsDroneAtt[2].plot(droneData.imu.time[-displayed_indices:], droneData.imu.az[-displayed_indices:], 
                               label='Z Accel', color='blue', alpha=0.7)
        axsDroneAtt[2].set_ylabel("m/s²")
        axsDroneAtt[2].set_title("Accelerations")
        axsDroneAtt[2].set_xlabel("Time (s)")
        axsDroneAtt[2].legend()
        axsDroneAtt[2].grid(True)
            
    # 3. Update Boat Attitude Window
    if enableBoatAttitudeWindow and len(axsBoatAtt) == 3:
        # Roll, Pitch, Yaw plot
        axsBoatAtt[0].clear()
        if boatData.simulation.roll:
            axsBoatAtt[0].plot(boatData.simulation.time[-displayed_indices:], boatData.simulation.roll[-displayed_indices:], 
                              label='Roll (SIM)', color='red')
            axsBoatAtt[0].plot(boatData.simulation.time[-displayed_indices:], boatData.simulation.pitch[-displayed_indices:], 
                              label='Pitch (SIM)', color='green')
            axsBoatAtt[0].plot(boatData.simulation.time[-displayed_indices:], boatData.simulation.yaw[-displayed_indices:], 
                              label='Yaw (SIM)', color='blue')
        elif boatData.attitude.roll:
            axsBoatAtt[0].plot(boatData.attitude.time[-displayed_indices:], boatData.attitude.roll[-displayed_indices:], 
                              label='Roll', color='red', alpha=0.7)
            axsBoatAtt[0].plot(boatData.attitude.time[-displayed_indices:], boatData.attitude.pitch[-displayed_indices:], 
                              label='Pitch', color='green', alpha=0.7)
            axsBoatAtt[0].plot(boatData.attitude.time[-displayed_indices:], boatData.attitude.yaw[-displayed_indices:], 
                              label='Yaw', color='blue', alpha=0.7)
        axsBoatAtt[0].set_ylabel("Degrees")
        axsBoatAtt[0].set_title("Attitude")
        axsBoatAtt[0].legend()
        axsBoatAtt[0].grid(True)
        
        # Angular rates plot
        axsBoatAtt[1].clear()
        if boatData.simulation.xgyro:
            axsBoatAtt[1].plot(boatData.simulation.time[-displayed_indices:], boatData.simulation.xgyro[-displayed_indices:], 
                              label='Roll rate (SIM)', color='red')
            axsBoatAtt[1].plot(boatData.simulation.time[-displayed_indices:], boatData.simulation.ygyro[-displayed_indices:], 
                              label='Pitch rate (SIM)', color='green')
            axsBoatAtt[1].plot(boatData.simulation.time[-displayed_indices:], boatData.simulation.zgyro[-displayed_indices:], 
                              label='Yaw rate (SIM)', color='blue')
        elif boatData.attitude.roll_speed:
            axsBoatAtt[1].plot(boatData.attitude.time[-displayed_indices:], boatData.attitude.roll_speed[-displayed_indices:], 
                              label='Roll rate', color='red', alpha=0.7)
            axsBoatAtt[1].plot(boatData.attitude.time[-displayed_indices:], boatData.attitude.pitch_speed[-displayed_indices:], 
                              label='Pitch rate', color='green', alpha=0.7)
            axsBoatAtt[1].plot(boatData.attitude.time[-displayed_indices:], boatData.attitude.yaw_speed[-displayed_indices:], 
                              label='Yaw rate', color='blue', alpha=0.7)
        axsBoatAtt[1].set_ylabel("Degrees/sec")
        axsBoatAtt[1].set_title("Angular Rates")
        axsBoatAtt[1].legend()
        axsBoatAtt[1].grid(True)
        
        # Accelerations plot
        axsBoatAtt[2].clear()
        if boatData.simulation.xacc:
            axsBoatAtt[2].plot(boatData.simulation.time[-displayed_indices:], boatData.simulation.xacc[-displayed_indices:], 
                              label='X Accel (SIM)', color='red')
            axsBoatAtt[2].plot(boatData.simulation.time[-displayed_indices:], boatData.simulation.yacc[-displayed_indices:], 
                              label='Y Accel (SIM)', color='green')
            axsBoatAtt[2].plot(boatData.simulation.time[-displayed_indices:], boatData.simulation.zacc[-displayed_indices:], 
                              label='Z Accel (SIM)', color='blue')
        elif boatData.imu.ax:
            axsBoatAtt[2].plot(boatData.imu.time[-displayed_indices:], boatData.imu.ax[-displayed_indices:], 
                              label='X Accel', color='red', alpha=0.7)
            axsBoatAtt[2].plot(boatData.imu.time[-displayed_indices:], boatData.imu.ay[-displayed_indices:], 
                              label='Y Accel', color='green', alpha=0.7)
            axsBoatAtt[2].plot(boatData.imu.time[-displayed_indices:], boatData.imu.az[-displayed_indices:], 
                              label='Z Accel', color='blue', alpha=0.7)
        axsBoatAtt[2].set_ylabel("m/s²")
        axsBoatAtt[2].set_title("Accelerations")
        axsBoatAtt[2].set_xlabel("Time (s)")
        axsBoatAtt[2].legend()
        axsBoatAtt[2].grid(True)
    
    # 4. Update Drone Velocity Window
    if enableDroneVelocityWindow and len(axsDroneVel) == 3:
        # X velocity
        axsDroneVel[0].clear()
        if droneData.simulation.vn:
            axsDroneVel[0].plot(droneData.simulation.time[-displayed_indices:], droneData.simulation.vn[-displayed_indices:], 
                               label='North Velocity (SIM)', color='red')
        elif droneData.position.vx:
            axsDroneVel[0].plot(droneData.position.time[-displayed_indices:], droneData.position.vx[-displayed_indices:], 
                               label='X Velocity', color='red', alpha=0.7)
        axsDroneVel[0].set_ylabel("m/s")
        axsDroneVel[0].set_title("North/X Velocity")
        axsDroneVel[0].legend()
        axsDroneVel[0].grid(True)
        
        # Y velocity
        axsDroneVel[1].clear()
        if droneData.simulation.ve:
            axsDroneVel[1].plot(droneData.simulation.time[-displayed_indices:], droneData.simulation.ve[-displayed_indices:], 
                               label='East Velocity (SIM)', color='green')
        elif droneData.position.vy:
            axsDroneVel[1].plot(droneData.position.time[-displayed_indices:], droneData.position.vy[-displayed_indices:], 
                               label='Y Velocity', color='green', alpha=0.7)
        axsDroneVel[1].set_ylabel("m/s")
        axsDroneVel[1].set_title("East/Y Velocity")
        axsDroneVel[1].legend()
        axsDroneVel[1].grid(True)
        
        # Z velocity
        axsDroneVel[2].clear()
        if droneData.simulation.vd:
            axsDroneVel[2].plot(droneData.simulation.time[-displayed_indices:], droneData.simulation.vd[-displayed_indices:], 
                               label='Down Velocity (SIM)', color='blue')
        elif droneData.position.vz:
            axsDroneVel[2].plot(droneData.position.time[-displayed_indices:], droneData.position.vz[-displayed_indices:], 
                               label='Z Velocity', color='blue', alpha=0.7)
        axsDroneVel[2].set_ylabel("m/s")
        axsDroneVel[2].set_xlabel("Time (s)")
        axsDroneVel[2].set_title("Down/Z Velocity")
        axsDroneVel[2].legend()
        axsDroneVel[2].grid(True)
    
    # 5. Update Boat Velocity Window
    if enableBoatVelocityWindow and len(axsBoatVel) == 3:
        # X velocity
        axsBoatVel[0].clear()
        if boatData.simulation.vn:
            axsBoatVel[0].plot(boatData.simulation.time[-displayed_indices:], boatData.simulation.vn[-displayed_indices:], 
                              label='North Velocity (SIM)', color='red')
        elif boatData.position.vx:
            axsBoatVel[0].plot(boatData.position.time[-displayed_indices:], boatData.position.vx[-displayed_indices:], 
                              label='X Velocity', color='red', alpha=0.7)
        axsBoatVel[0].set_ylabel("m/s")
        axsBoatVel[0].set_title("North/X Velocity")
        axsBoatVel[0].legend()
        axsBoatVel[0].grid(True)
        
        # Y velocity
        axsBoatVel[1].clear()
        if boatData.simulation.ve:
            axsBoatVel[1].plot(boatData.simulation.time[-displayed_indices:], boatData.simulation.ve[-displayed_indices:], 
                              label='East Velocity (SIM)', color='green')
        elif boatData.position.vy:
            axsBoatVel[1].plot(boatData.position.time[-displayed_indices:], boatData.position.vy[-displayed_indices:], 
                              label='Y Velocity', color='green', alpha=0.7)
        axsBoatVel[1].set_ylabel("m/s")
        axsBoatVel[1].set_title("East/Y Velocity")
        axsBoatVel[1].legend()
        axsBoatVel[1].grid(True)
        
        # Z velocity
        axsBoatVel[2].clear()
        if boatData.simulation.vd:
            axsBoatVel[2].plot(boatData.simulation.time[-displayed_indices:], boatData.simulation.vd[-displayed_indices:], 
                              label='Down Velocity (SIM)', color='blue')
        elif boatData.position.vz:
            axsBoatVel[2].plot(boatData.position.time[-displayed_indices:], boatData.position.vz[-displayed_indices:], 
                              label='Z Velocity', color='blue', alpha=0.7)
        axsBoatVel[2].set_ylabel("m/s")
        axsBoatVel[2].set_xlabel("Time (s)")
        axsBoatVel[2].set_title("Down/Z Velocity")
        axsBoatVel[2].legend()
        axsBoatVel[2].grid(True)
    
    # 6. Update Relative Velocity Window
    if enableRelativeVelocityWindow and len(axsRelVel) == 4:
        # Create relative velocity time history
        rel_vn_history = []
        rel_ve_history = []
        rel_vd_history = []
        rel_v_mag_history = []
        time_history = []
        
        # Generate relative velocity data points from simulation or position data
        # First, try to use simulation data
        # if droneData.simulation.vn and boatData.simulation.vn:
        #     for i in range(min(len(droneData.simulation.time), len(boatData.simulation.time))):
        #         vn_rel = droneData.simulation.vn[i] - boatData.simulation.vn[i]
        #         ve_rel = droneData.simulation.ve[i] - boatData.simulation.ve[i]
        #         vd_rel = droneData.simulation.vd[i] - boatData.simulation.vd[i]
        #         v_mag = get_vector_magnitude(vn_rel, ve_rel, vd_rel)
                
        #         rel_vn_history.append(vn_rel)
        #         rel_ve_history.append(ve_rel)
        #         rel_vd_history.append(vd_rel)
        #         rel_v_mag_history.append(v_mag)
        #         time_history.append(droneData.simulation.time[i])
        # If no simulation data, try position data
        if droneData.gps.time and boatData.gps.time:
            for i in range(min(len(droneData.position.time), len(boatData.position.time))):
                vx_rel = droneData.gps.vx[i] - boatData.gps.vx[i]
                vy_rel = droneData.gps.vy[i] - boatData.gps.vy[i]
                vz_rel = droneData.gps.vz[i] - boatData.gps.vz[i]
                v_drone = np.sqrt(droneData.gps.vx[i]**2 + droneData.gps.vy[i]**2)
                v_boat = np.sqrt(boatData.gps.vx[i]**2 + boatData.gps.vy[i]**2)
                v_mag = v_boat-v_drone

                
                rel_vn_history.append(vx_rel)
                rel_ve_history.append(vy_rel)
                rel_vd_history.append(vz_rel)
                rel_v_mag_history.append(v_mag)
                time_history.append(droneData.position.time[i])
        
        # Plot the relative velocity data
        if time_history:
            # North relative velocity
            axsRelVel[0].clear()
            axsRelVel[0].plot(time_history[-displayed_indices:], rel_vn_history[-displayed_indices:], label='North V', color='red')
            axsRelVel[0].set_ylabel("m/s")
            axsRelVel[3].set_xlabel("Time (s)")
            axsRelVel[0].set_title("North Relative Velocity (Drone-Boat)")
            axsRelVel[0].legend()
            axsRelVel[0].grid(True)
            
            # East relative velocity
            axsRelVel[1].clear()
            axsRelVel[1].plot(time_history[-displayed_indices:], rel_ve_history[-displayed_indices:], label='East V', color='green')
            axsRelVel[1].set_ylabel("m/s")
            axsRelVel[3].set_xlabel("Time (s)")
            axsRelVel[1].set_title("North Relative Velocity (Drone-Boat)")
            axsRelVel[1].legend()
            axsRelVel[1].grid(True)
            
            # Down relative velocity
            axsRelVel[2].clear()
            axsRelVel[2].plot(time_history[-displayed_indices:], rel_vd_history[-displayed_indices:], label='Z V', color='blue')
            axsRelVel[2].set_ylabel("m/s")
            axsRelVel[3].set_xlabel("Time (s)")
            axsRelVel[2].set_title("Z Relative Velocity (Drone-Boat)")
            axsRelVel[2].legend()
            axsRelVel[2].grid(True)
            
            # Absolute velocity magnitude
            axsRelVel[3].clear()
            axsRelVel[3].plot(time_history[-displayed_indices:], rel_v_mag_history[-displayed_indices:], label='Velocity Magnitude (planar)', color='purple')
            axsRelVel[3].set_ylabel("m/s")
            axsRelVel[3].set_xlabel("Time (s)")
            axsRelVel[3].set_title("Absolute Velocity Diffrence (Planar)")
            axsRelVel[3].legend()
            axsRelVel[3].grid(True)
    
    # 7. Update Wind Window
    if enableWindWindow and len(axsWind) == 2:
        # Wind speed
        axsWind[0].clear()
        if droneData.wind.speed:  # Assuming wind data comes from boat
            axsWind[0].plot(droneData.wind.time[-displayed_indices:], droneData.wind.speed[-displayed_indices:], label='Wind Speed', color='cyan')
            axsWind[0].set_title("Wind Speed")
            axsWind[0].set_ylabel("m/s")
            axsWind[0].legend()
            axsWind[0].grid(True)
        
        # Wind direction
        axsWind[1].clear()
        if droneData.wind.direction:
            axsWind[1].plot(droneData.wind.time[-displayed_indices:], droneData.wind.direction[-displayed_indices:], label='Wind Direction', color='magenta')
            axsWind[1].set_title("Wind Direction")
            axsWind[1].set_ylabel("Degrees")
            axsWind[1].set_xlabel("Time (s)")
            axsWind[1].legend()
            axsWind[1].grid(True)


    # 8. Update Collision Window
    if enableCollisionWindow:
        # Collision detection
        axsCollision[0, 0].clear()
        axsCollision[0, 0].plot(collision_data.time[-displayed_indices:], collision_data.distance[-displayed_indices:], label='Absolute distance', color='red')
        axsCollision[0, 0].set_title("Absolute Distance, boat to drone")
        axsCollision[0, 0].set_ylabel("Collision")
        axsCollision[0, 0].legend()
        axsCollision[0, 0].grid(True)
        
        # Collision distance
        axsCollision[0, 1].clear()
        axsCollision[0, 1].plot(collision_data.time[-displayed_indices:], collision_data.delta_x[-displayed_indices:], label='Delta X', color='blue')
        axsCollision[0, 1].set_title("Delta X to Collision")
        axsCollision[0, 1].set_ylabel("Distance (m)")
        axsCollision[0, 1].legend()
        axsCollision[0, 1].grid(True)
        
        # Collision delta time
        axsCollision[1, 0].clear()
        axsCollision[1, 0].plot(collision_data.time[-displayed_indices:], collision_data.delta_y[-displayed_indices:], label='Delta Y', color='green')
        axsCollision[1, 0].set_title("Delta Y to Collision")
        axsCollision[1, 0].set_ylabel("Distance (m)")
        axsCollision[1, 0].legend()
        axsCollision[1, 0].grid(True)
        
        # Collision delta x
        axsCollision[1, 1].clear()
        axsCollision[1, 1].plot(collision_data.time[-displayed_indices:], collision_data.delta_z[-displayed_indices:], label='Delta Z', color='orange')
        axsCollision[1, 1].set_title("Delta Z to Collision")
        axsCollision[1, 1].set_ylabel("Distance (m)")
        axsCollision[1, 1].legend()
        axsCollision[1, 1].grid(True)

        # Delta time
        axsCollision[2, 0].clear()
        axsCollision[2, 0].plot(collision_data.time[-displayed_indices:], collision_data.delta_time[-displayed_indices:], label='Delta Time', color='purple')
        axsCollision[2, 0].set_title("Delta Time measurments")
        axsCollision[2, 0].set_ylabel("Time (s)")
        axsCollision[2, 0].set_xlabel("Timesteps")


    if enableDistToP:

        # Distance to P1
        axsDistToP[0].clear()
        if callbacks.P1:
            axsDistToP[0].plot(callbacks.P1.time[-(int(displayed_indices/3)):], callbacks.P1.dist[-(int(displayed_indices/3)):], label='Distance to P1', color=colors.p1)
            axsDistToP[0].set_title("Distance to P1")
            axsDistToP[0].set_ylabel("Distance (m)")
            axsDistToP[0].legend()
            axsDistToP[0].grid(True)
        
        # Distance to P2
        axsDistToP[1].clear()
        if callbacks.P2:
            axsDistToP[1].plot(callbacks.P2.time[-(int(displayed_indices/3)):], callbacks.P2.dist[-(int(displayed_indices/3)):], label='Distance to P2', color=colors.p2)
            axsDistToP[1].set_title("Distance to P2")
            axsDistToP[1].set_ylabel("Distance (m)")
            axsDistToP[1].legend()
            axsDistToP[1].grid(True)
        
        # Distance to P3
        axsDistToP[2].clear()
        if callbacks.P3:
            axsDistToP[2].plot(callbacks.P3.time[-(int(displayed_indices/3)):], callbacks.P3.dist[-(int(displayed_indices/3)):], label='Distance to P3', color=colors.p3)
            axsDistToP[2].set_title("Distance to P3")
            axsDistToP[2].set_ylabel("Distance (m)")
            axsDistToP[2].legend()
            axsDistToP[2].grid(True)


# Show all plots non-blocking
plt.tight_layout()
plt.ion()  # Enable interactive mode
plt.show(block=False)

# Main update loop
try:
    while True:
        update_plot(0)
        plt.pause(0.2)
except KeyboardInterrupt:
    # Save plots if requested
    print("Exiting...")

    if savePlots:
        timestamp = time.strftime('%Y%m%d_%H%M%S')
        if enableGlobalWindow:
            figGlobal.savefig(f"plots/global_position_{timestamp}.png")
        if enableDroneAttitudeWindow:
            figDroneAtt.savefig(f"plots/drone_attitude_{timestamp}.png")
        if enableBoatAttitudeWindow:
            figBoatAtt.savefig(f"plots/boat_attitude_{timestamp}.png")
        if enableDroneVelocityWindow:
            figDroneVel.savefig(f"plots/drone_velocity_{timestamp}.png")
        if enableBoatVelocityWindow:
            figBoatVel.savefig(f"plots/boat_velocity_{timestamp}.png")
        if enableRelativeVelocityWindow:
            figRelVel.savefig(f"plots/relative_velocity_{timestamp}.png")
        if enableWindWindow:
            figWind.savefig(f"plots/wind_data_{timestamp}.png")
        print("Plots saved.")
    else:
        print("No plots saved")    
    # Close all plot windows
    plt.close('all')
    exit(0)