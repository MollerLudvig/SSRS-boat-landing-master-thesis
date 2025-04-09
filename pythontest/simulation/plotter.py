import matplotlib.pyplot as plt
from redis_communication import RedisClient
from time import sleep

rc = RedisClient()

plt.ion()
fig, ax = plt.subplots()
line1, = ax.plot([], [], 'bo-', label = 'Drone altitude')
line2, = ax.plot([], [], 'go--', label='Z Wanted')

ax.legend()

# Drone altitude lags behind a lot, when drone.altitude says it is at 12m 
# the console from mavlink connection says it is at 8m 
# Maybe just saving values and plotting after will help?
def plot_trajectory():

    try:
        [xs, altitudes, boat_altitude, z_wanteds] = rc.get_latest_stream_message("traj_plot")[1]
        Gr = rc.get_latest_stream_message("glide_ratio")[1]

        hline = ax.axhline(y=boat_altitude, color='red', linestyle='--', label='Boat altitude')
        
        line1.set_xdata(xs)
        line1.set_ydata(altitudes)
        ax.set_xlabel("Distance since descent start (m)")

        line2.set_xdata(xs)
        line2.set_ydata(z_wanteds)
        ax.set_ylabel("Altitude (m)")

        # Rescale axes
        ax.relim()
        ax.autoscale_view()
        ax.set_title(f"Glide ratio: 1/{int(1/Gr)}")

        plt.draw()
        plt.pause(0.01)
    except:
        print("Waiting for stream")
    
    sleep(1)

while True:
    plot_trajectory()