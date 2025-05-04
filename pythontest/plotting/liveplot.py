import matplotlib.pyplot as plt
from pymavlink import mavutil
import time
from VehicleMonitor import VehicleMonitor
from variabels import Variables

enableDroneWindow = True
enableBoatWindow = True
enableGlobalWindow = True
savePlots = False

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

# Create main figure
fig, axs = plt.subplots(3, 2, figsize=(12, 10))
(axPosDrone, axAttDrone), (axPosBoat, axAttBoat), (axGlobalPos, _) = axs
fig.suptitle("Live MAVLink Vehicle Monitor", fontsize=16)
axs[2][1].axis('off')

droneData = Variables(drone)
boatData = Variables(boat)

def update_plot(_):
    # Drone
    posD, attD, gpsD, velD, accD, angD, windD, simD = drone.get_latest_data()

    droneData.set_data()
    boatData.set_data()

    if posD:
        _, x, y, *_ = zip(*posD)
        axPosDrone.clear()
        axPosDrone.plot(x, y, color=drone.color)
        axPosDrone.set_title("Drone Position (X, Y)")
        axPosDrone.axis('equal')

    if attD:
        _, roll, pitch, yaw, *_ = zip(*attD)
        axAttDrone.clear()
        axAttDrone.plot(roll, label='Roll')
        axAttDrone.plot(pitch, label='Pitch')
        axAttDrone.plot(yaw, label='Yaw')
        axAttDrone.set_title("Drone Attitude")
        axAttDrone.legend()

    # Boat
    posB, attB, gpsB, velB, accB, angB, windB, simB = boat.get_latest_data()
    if posB:
        _, x, y, *_ = zip(*posB)
        axPosBoat.clear()
        axPosBoat.plot(x, y, color=boat.color)
        axPosBoat.set_title("Boat Position (X, Y)")
        axPosBoat.axis('equal')

    if attB:
        _, roll, pitch, yaw, *_ = zip(*attB)
        axAttBoat.clear()
        axAttBoat.plot(roll, label='Roll')
        axAttBoat.plot(pitch, label='Pitch')
        axAttBoat.plot(yaw, label='Yaw')
        axAttBoat.set_title("Boat Attitude")
        axAttBoat.legend()

    # Global
    axGlobalPos.clear()
    if gpsD:
        _, latD, lonD, *_ = zip(*gpsD)
        axGlobalPos.plot(lonD, latD, label="Drone", color=drone.color)
    if gpsB:
        _, latB, lonB, *_ = zip(*gpsB)
        axGlobalPos.plot(lonB, latB, label="Boat", color=boat.color)
    axGlobalPos.set_title("Global Position (Lat, Lon)")
    axGlobalPos.set_xlabel("Longitude")
    axGlobalPos.set_ylabel("Latitude")
    axGlobalPos.legend()
    axGlobalPos.axis('equal')

    # Update extra figures if enabled
    if enableDroneWindow:
        axDronePos.clear()
        if posD:
            _, xD, yD, *_ = zip(*posD)
            axDronePos.plot(xD, yD, color=drone.color)
            axDronePos.set_title("Drone (X, Y)")
        axDroneAtt.clear()
        if attD:
            _, rollD, pitchD, yawD, *_ = zip(*attD)
            axDroneAtt.plot(rollD, label='Roll')
            axDroneAtt.plot(pitchD, label='Pitch')
            axDroneAtt.plot(yawD, label='Yaw')
            axDroneAtt.set_title("Drone Attitude")
            axDroneAtt.legend()

    if enableBoatWindow:
        axBoatPos.clear()
        if posB:
            _, xB, yB, *_ = zip(*posB)
            axBoatPos.plot(xB, yB, color=boat.color)
            axBoatPos.set_title("Boat (X, Y)")
        axBoatAtt.clear()
        if attB:
            _, rollB, pitchB, yawB, *_ = zip(*attB)
            axBoatAtt.plot(rollB, label='Roll')
            axBoatAtt.plot(pitchB, label='Pitch')
            axBoatAtt.plot(yawB, label='Yaw')
            axBoatAtt.set_title("Boat Attitude")
            axBoatAtt.legend()


    if enableGlobalWindow:
        axGlobal.clear()
        if gpsD:
            axGlobal.plot(lonD, latD, label='Drone', color=drone.color)
        if gpsB:
            axGlobal.plot(lonB, latB, label='Boat', color=boat.color)
        axGlobal.set_title("Global Position (Lat, Lon)")
        axGlobal.set_xlabel("Longitude")
        axGlobal.set_ylabel("Latitude")
        axGlobal.legend()
        axGlobal.axis('equal')

# Show all plots non-blocking
plt.tight_layout()
plt.show(block=False)

# Main update loop
try:
    while True:
        update_plot(0)
        plt.pause(0.2)
except KeyboardInterrupt:
    # Save plots plt
    print("Exiting...")
    if savePlots:
        fig.savefig(f"main_plot_{time.strftime('%Y%m%d_%H%M%S')}.png")
        if enableDroneWindow:
            figDrone.savefig(f"drone_plot{drone.name}_{time.strftime('%Y%m%d_%H%M%S')}.png")
        if enableBoatWindow:
            figBoat.savefig(f"boat_plot{boat.name}_{time.strftime('%Y%m%d_%H%M%S')}.png")
        if enableGlobalWindow:
            figGlobal.savefig(f"global_plot{time.strftime('%Y%m%d_%H%M%S')}.png")
        plt.close(fig)
        if enableDroneWindow:
            plt.close(figDrone)

        if enableBoatWindow:
            plt.close(figBoat)
        if enableGlobalWindow:
            plt.close(figGlobal)
    else:
        plt.close(fig)
        if enableDroneWindow:
            plt.close(figDrone)
        if enableBoatWindow:
            plt.close(figBoat)
        if enableGlobalWindow:
            plt.close(figGlobal)
    print("Plots saved.")
    exit(0)

