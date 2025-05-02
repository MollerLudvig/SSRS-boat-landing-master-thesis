import matplotlib.pyplot as plt
import matplotlib.animation as animation
from pymavlink import mavutil
from threading import Thread, Lock
import time
import math
from VehicleMonitor import VehicleMonitor

enableDroneWindow = True
enableBoatWindow = True
enableGlobalWindow = True


# Initialize vehicles
drone = VehicleMonitor(name="Drone", udpPort=14551, color='blue')
boat = VehicleMonitor(name="Boat", udpPort=14561, color='green')
                      

# Create figures and axes conditionally
if enableDroneWindow:
    figDrone, (axDronePos, axDroneAtt) = plt.subplots(2, 1, num="Drone View")
if enableBoatWindow:
    figBoat, (axBoatPos, axBoatAtt) = plt.subplots(2, 1, num="Boat View")
if enableGlobalWindow:
    figGlobal, axGlobal = plt.subplots(1, 1, num="Global Position")



# Create plots
fig, axs = plt.subplots(3, 2, figsize=(12, 10))
(axPosDrone, axAttDrone), (axPosBoat, axAttBoat), (axGlobalPos, _) = axs
fig.suptitle("Live MAVLink Vehicle Monitor", fontsize=16)
axs[2][1].axis('off')  # Hide unused plot

def update_plot(frame):
    # Drone
    posD, attD, gpsD, velD, accD, angD, windD, simD = drone.get_latest_data()
    if posD:
        x, y = zip(*posD)
        axPosDrone.clear()
        axPosDrone.plot(x, y, color=drone.color)
        axPosDrone.set_title("Drone Position (X, Y)")
        axPosDrone.axis('equal')

    if attD:
        roll, pitch, yaw = zip(*attD)
        axAttDrone.clear()
        axAttDrone.plot(roll, label='Roll')
        axAttDrone.plot(pitch, label='Pitch')
        axAttDrone.plot(yaw, label='Yaw')
        axAttDrone.set_title("Drone Attitude")
        axAttDrone.legend()

    # Boat
    posB, attB, gpsB, velB, accB, angB, windB, simB  = boat.get_latest_data()

    if posB:
        x, y = zip(*posB)
        axPosBoat.clear()
        axPosBoat.plot(x, y, color=boat.color)
        axPosBoat.set_title("Boat Position (X, Y)")
        axPosBoat.axis('equal')

    if attB:
        roll, pitch, yaw = zip(*attB)
        axAttBoat.clear()
        axAttBoat.plot(roll, label='Roll')
        axAttBoat.plot(pitch, label='Pitch')
        axAttBoat.plot(yaw, label='Yaw')
        axAttBoat.set_title("Boat Attitude")
        axAttBoat.legend()

    # Global Position Plot
    axGlobalPos.clear()
    if gpsD:
        latD, lonD = zip(*gpsD)
        axGlobalPos.plot(lonD, latD, label="Drone", color=drone.color)
    if gpsB:
        latB, lonB = zip(*gpsB)
        axGlobalPos.plot(lonB, latB, label="Boat", color=boat.color)
    axGlobalPos.set_title("Global Position (Lat, Lon)")
    axGlobalPos.set_xlabel("Longitude")
    axGlobalPos.set_ylabel("Latitude")
    axGlobalPos.legend()
    axGlobalPos.axis('equal')


ani = animation.FuncAnimation(fig, update_plot, interval=200)
plt.tight_layout()
plt.show()
