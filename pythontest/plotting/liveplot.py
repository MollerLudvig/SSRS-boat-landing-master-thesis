import matplotlib.pyplot as plt
from pymavlink import mavutil
import time
from VehicleMonitor import VehicleMonitor
from variabels import VehicleData  # Import the new VehicleData class

enableDroneWindow = True
enableBoatWindow = True
enableGlobalWindow = True
savePlots = False

# Initialize vehicles
drone = VehicleMonitor(name="Drone", udpPort=14551, color='blue')
boat = VehicleMonitor(name="Boat", udpPort=14561, color='green')

# Initialize data containers with the new VehicleData class
droneData = VehicleData(drone)
boatData = VehicleData(boat)

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

def update_plot(_):
    # Update data containers (thread-safe)
    droneData.update()
    boatData.update()

    # Drone position plot
    if droneData.position.x:
        axPosDrone.clear()
        axPosDrone.plot(droneData.position.x, droneData.position.y, color=drone.color)
        axPosDrone.set_title("Drone Position (X, Y)")
        axPosDrone.axis('equal')

    # Drone attitude plot
    if droneData.attitude.roll:
        axAttDrone.clear()
        axAttDrone.plot(droneData.attitude.roll, label='Roll')
        axAttDrone.plot(droneData.attitude.pitch, label='Pitch')
        axAttDrone.plot(droneData.attitude.yaw, label='Yaw')
        axAttDrone.set_title("Drone Attitude")
        axAttDrone.legend()

    # Boat position plot
    if boatData.position.x:
        axPosBoat.clear()
        axPosBoat.plot(boatData.position.x, boatData.position.y, color=boat.color)
        axPosBoat.set_title("Boat Position (X, Y)")
        axPosBoat.axis('equal')

    # Boat attitude plot
    if boatData.attitude.roll:
        axAttBoat.clear()
        axAttBoat.plot(boatData.attitude.roll, label='Roll')
        axAttBoat.plot(boatData.attitude.pitch, label='Pitch')
        axAttBoat.plot(boatData.attitude.yaw, label='Yaw')
        axAttBoat.set_title("Boat Attitude")
        axAttBoat.legend()

    # Global position plot
    axGlobalPos.clear()
    if droneData.gps.lat:
        axGlobalPos.plot(droneData.gps.lon, droneData.gps.lat, label="Drone", color=drone.color)
    if boatData.gps.lat:
        axGlobalPos.plot(boatData.gps.lon, boatData.gps.lat, label="Boat", color=boat.color)
    axGlobalPos.set_title("Global Position (Lat, Lon)")
    axGlobalPos.set_xlabel("Longitude")
    axGlobalPos.set_ylabel("Latitude")
    axGlobalPos.legend()
    axGlobalPos.axis('equal')

    # Update extra figures if enabled
    if enableDroneWindow:
        axDronePos.clear()
        if droneData.position.x:
            axDronePos.plot(droneData.position.x, droneData.position.y, color=drone.color)
            axDronePos.set_title("Drone (X, Y)")
        
        axDroneAtt.clear()
        if droneData.attitude.roll:
            axDroneAtt.plot(droneData.attitude.roll, label='Roll')
            axDroneAtt.plot(droneData.attitude.pitch, label='Pitch')
            axDroneAtt.plot(droneData.attitude.yaw, label='Yaw')
            axDroneAtt.set_title("Drone Attitude")
            axDroneAtt.legend()

    if enableBoatWindow:
        axBoatPos.clear()
        if boatData.position.x:
            axBoatPos.plot(boatData.position.x, boatData.position.y, color=boat.color)
            axBoatPos.set_title("Boat (X, Y)")
            
        axBoatAtt.clear()
        if boatData.attitude.roll:
            axBoatAtt.plot(boatData.attitude.roll, label='Roll')
            axBoatAtt.plot(boatData.attitude.pitch, label='Pitch')
            axBoatAtt.plot(boatData.attitude.yaw, label='Yaw')
            axBoatAtt.set_title("Boat Attitude")
            axBoatAtt.legend()

    if enableGlobalWindow:
        axGlobal.clear()
        if droneData.gps.lat:
            axGlobal.plot(droneData.gps.lon, droneData.gps.lat, label='Drone', color=drone.color)
        if boatData.gps.lat:
            axGlobal.plot(boatData.gps.lon, boatData.gps.lat, label='Boat', color=boat.color)
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
    # Save plots if requested
    print("Exiting...")
    if savePlots:
        fig.savefig(f"main_plot_{time.strftime('%Y%m%d_%H%M%S')}.png")
        if enableDroneWindow:
            figDrone.savefig(f"drone_plot{drone.name}_{time.strftime('%Y%m%d_%H%M%S')}.png")
        if enableBoatWindow:
            figBoat.savefig(f"boat_plot{boat.name}_{time.strftime('%Y%m%d_%H%M%S')}.png")
        if enableGlobalWindow:
            figGlobal.savefig(f"global_plot{time.strftime('%Y%m%d_%H%M%S')}.png")
    
    # Close all plot windows
    plt.close(fig)
    if enableDroneWindow:
        plt.close(figDrone)
    if enableBoatWindow:
        plt.close(figBoat)
    if enableGlobalWindow:
        plt.close(figGlobal)
        
    print("Plots saved.")
    exit(0)