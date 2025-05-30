import numpy as np
import pandas as pd
import csv

# import matplotlib
# matplotlib.use("Agg")  # Use non-interactive backend
import matplotlib.pyplot as plt

import cartopy.crs as ccrs
import cartopy.feature as cfeature
from cartopy.io.img_tiles import OSM
from kalman_OOSM import KalmanFilterXY
from coordinate_conv import latlon_to_xy, xy_to_latlon, ned_to_latlon, latlon_to_ned

# Load AIS data
# csv_file = "pythontest/Guidance/valo_3.csv"  
# csv_file = "valo_3.csv" 
# csv_file = "simulation/Guidance/data_short2_modded.csv"
# csv_file = "data_short_short.csv"
# csv_file = "data_short2.csv"
csv_file = "ALV_SNABBEN_5_1.csv"

df = pd.read_csv(csv_file)

# Get initial reference position
lat0, lon0, v0, psi0, time0 = df.iloc[0]["lat"], df.iloc[0]["lon"], df.iloc[0]["speed[m/s]"], df.iloc[0]["heading"], df.iloc[0]["timestamp_unix"]

time_serises = df["timestamp_unix"].to_numpy()
time_serises -= time_serises[0]  # Normalize timestamps to start from 0
lastTime = time_serises[-1]  # Get the last timestamp
time0 = time_serises[0]  # Set the initial time to the first timestamp

# Convert lat/lon to local XY
df["x"], df["y"] = zip(*df.apply(lambda row: latlon_to_ned(row["lat"], row["lon"], lat0, lon0), axis=1))

dfPlot = df.copy()


print(dfPlot[["x", "y", "speed[m/s]"]].head(10))

heading_rad = np.radians(df.iloc[0]["heading"])
course_rad = np.radians(df.iloc[0]["course"])

delta_r = course_rad - heading_rad
# Normalize delta_r to be within -pi to pi
delta_r = (delta_r + np.pi) % (2*np.pi) - np.pi

u = df.iloc[0]["speed[m/s]"] * np.cos(delta_r)
v = df.iloc[0]["speed[m/s]"] * np.sin(delta_r)


print(f"Initial velocity: {u}, {v}")

kf = KalmanFilterXY(u = u, v = v, heading=psi0, init_lat=lat0, init_lon=lon0, timestamp = time0,  process_noise_variance=0.001)

t = 0
dt = 1  # Time step in seconds
t += dt
trajectory = []
lats = []
lons = []
u_vel = []
v_vel = []
velocity = []
heading = []
course = []
yawrate = []
ax = []
ay = []
savedStates = []
savedMeasurements = []
timestamps = []
delta_time = []
def wrap_to_180(angle_deg):
    return (angle_deg + 180) % 360 - 180
time_index = 0

while True:
    # Update with all AIS measurements up to current time t
    while not df.empty and t >= time_serises[time_index]:

        # heading_rad = np.radians(df.iloc[0]["heading"])
        # course_rad = np.radians(df.iloc[0]["course"])

        # delta_r = course_rad - heading_rad
        # # Normalize delta_r to be within -pi to pi
        # delta_r = (delta_r + np.pi) % (2*np.pi) - np.pi

        # u = df.iloc[0]["speed[m/s]"] * np.cos(delta_r)
        # v = df.iloc[0]["speed[m/s]"] * np.sin(delta_r)

        # z = np.array([
        #     [df.iloc[0]["lat"]],
        #     [df.iloc[0]["lon"]],
        #     [df.iloc[0]["heading"]],
        #     [u],
        #     [v]
        # ])

        # measurment_time = df.iloc[0]["timestamp_unix"]
        # kf.update_w_latlon(z, measurment_time)

        z = np.array([
            [df.iloc[0]["lat"]],
            [df.iloc[0]["lon"]],
            [df.iloc[0]["heading"]],  # Heading in deg
            [df.iloc[0]["course"]],  # Course in deg
            [df.iloc[0]["speed[m/s]"]],  # Speed in m/s
        ])
        measurment_time = time_serises[time_index]

        # Save measurement
        savedMeasurements.append({
            "timestamp": measurment_time,
            "lat": z[0, 0],
            "lon": z[1, 0],
            "heading": z[2, 0],
            "course": z[3, 0],
            "speed": z[4, 0],
        })

        kf.update_AIS(z, measurment_time)

        df.drop(index=df.index[0], inplace=True)
        time_index += 1  # Increment the time index
        if time_index >= len(time_serises):
            break


    # Predict the state forward
    kf.predict_EKF(t)

    u = kf.x[3][0]
    v = kf.x[4][0]

    delta_time.append(t - measurment_time)
    trajectory.append((kf.x[0], kf.x[1]))
    lats.append(kf.lat)
    lons.append(kf.lon)
    velocity.append(np.sqrt(u**2 + v**2))
    u_vel.append(u)
    v_vel.append(v)
    course.append(np.rad2deg(np.arctan2(v,u)+kf.x[2, 0]) )
    heading.append(kf.x[2, 0])
    yawrate.append(kf.x[5, 0])
    ax.append(kf.x[6, 0])
    ay.append(kf.x[7, 0])
    timestamps.append(t)

    savedStates.append({
    "timestamp": t,
    "x": kf.x[0, 0],
    "y": kf.x[1, 0],
    "heading": np.rad2deg(kf.x[2, 0]),
    "u": kf.x[3, 0],
    "v": kf.x[4, 0],
    "yawrate": kf.x[5, 0],
    "ax": kf.x[6, 0],
    "ay": kf.x[7, 0],
    "lat": kf.lat,
    "lon": kf.lon
})


    # print(f"t: {t}, lat: {kf.lat}, lon: {kf.lon}, u: {u}, v: {v}, heading: {np.rad2deg(kf.x[2, 0])}, yawrate: {kf.x[5, 0]}")

    if t > lastTime:
        break
    t += dt


# Convert filtered XY back to lat/lon for plotting
filtered_lats, filtered_lons = zip(*[ned_to_latlon(x[0], x[1], lat0, lon0) for x in trajectory])
# filtered_lats = lats
# filtered_lons = lons

# print (f"Filtered trajectory: {filtered_lats}, {filtered_lons}")


# !-- Save results to CSV -----!
pd.DataFrame(savedStates).to_csv("ekf_states.csv", index=False)
pd.DataFrame(savedMeasurements).to_csv("ekf_measurements.csv", index=False)
print("Saved EKF states to ekf_states.csv and measurements to ekf_measurements.csv")


# !----- Plotting -----!
commonFigsize = (13, 6)
labelFontsize = 19
tickFontsize = 15
titleFontsize = 25
legendFontsize = 15


"""
Plot latlons
"""
plt.figure(figsize=commonFigsize)
plt.plot(filtered_lats, filtered_lons, 'b-', label="EKF Path")
plt.plot(dfPlot["lat"].to_numpy(), dfPlot["lon"].to_numpy(), 'ro', markersize=3, label="Raw AIS Data")
plt.xlabel("Latitude", fontsize=labelFontsize)
plt.ylabel("Longitude", fontsize=labelFontsize)
plt.xticks(fontsize=tickFontsize)
plt.yticks(fontsize=tickFontsize)
plt.title("Coordinates: AIS vs EKF", fontsize=titleFontsize)
plt.legend(fontsize=legendFontsize)
plt.grid()
plt.tight_layout()


"""
Plot filtered velocity
"""
plt.figure(figsize=commonFigsize)
plt.plot(timestamps, velocity, label="EKF Velocity", color="blue")
plt.plot(time_serises, dfPlot["speed[m/s]"].to_numpy(), label="Measured Velocity", color="red", alpha=0.6)
plt.plot(timestamps, u_vel, label="EKF u velocity", color="green", alpha=0.6)
plt.plot(timestamps, v_vel, label="EKF v velocity", color="orange", alpha=0.6)
plt.xlabel("Time [s]", fontsize=labelFontsize)
plt.ylabel("Velocity [m/s]", fontsize=labelFontsize)
plt.xticks(fontsize=tickFontsize)
plt.yticks(fontsize=tickFontsize)
plt.title("Velocity: AIS vs EKF", fontsize=titleFontsize)
plt.legend(fontsize=legendFontsize)
plt.grid(True)
plt.tight_layout()



"""
Plot filtered heading
"""
measurd_heading = wrap_to_180(dfPlot["heading"].to_numpy())
measurd_course = wrap_to_180(dfPlot["course"].to_numpy())

plt.figure(figsize=commonFigsize)
plt.plot(timestamps, np.rad2deg(heading), label="EKF Heading", color="blue")
plt.plot(timestamps, course, label="EKF Course", color="green", alpha=0.6)
plt.plot(time_serises, measurd_heading, label="Measured Heading", color="red", alpha=0.6)
plt.plot(time_serises, measurd_course, label="Measured Course", color="orange", alpha=0.6)
plt.xlabel("Time [s]", fontsize=labelFontsize)
plt.ylabel("Heading [deg]", fontsize=labelFontsize)
plt.xticks(fontsize=tickFontsize)
plt.yticks(fontsize=tickFontsize)
plt.title("Heading: AIS vs EKF", fontsize=titleFontsize)
plt.legend(fontsize=legendFontsize)
plt.grid(True)
plt.tight_layout()


"""
Plot timestams
"""
plt.figure(figsize=commonFigsize)
plt.plot(timestamps, label="Timestamps", color="blue")
plt.xlabel("Time [s]", fontsize=labelFontsize)
plt.ylabel("Timestamps [s]", fontsize=labelFontsize)
plt.xticks(fontsize=tickFontsize)
plt.yticks(fontsize=tickFontsize)
plt.title("Timestamps", fontsize=titleFontsize)
plt.legend(fontsize=legendFontsize)
plt.grid(True)
plt.tight_layout()


"""
Plot delta time
"""
plt.figure(figsize=commonFigsize)
plt.plot(timestamps, delta_time, label="Delta Time", color="blue")
plt.xlabel("Time [s]", fontsize=labelFontsize)
plt.ylabel("Delta Time [s]", fontsize=labelFontsize)
plt.xticks(fontsize=tickFontsize)
plt.yticks(fontsize=tickFontsize)
plt.title("Delta Time", fontsize=titleFontsize)
plt.legend(fontsize=legendFontsize)
plt.grid(True)
plt.tight_layout()



"""
Plot filtered yawrate
"""
plt.figure(figsize=commonFigsize)
plt.plot(timestamps, yawrate, label="EKF Yawrate", color="blue")
plt.xlabel("Time [s]", fontsize=labelFontsize)
plt.ylabel("Yawrate [rad/s]", fontsize=labelFontsize)
plt.xticks(fontsize=tickFontsize)
plt.yticks(fontsize=tickFontsize)
plt.title("Yawrate EKF", fontsize=titleFontsize)
plt.legend(fontsize=legendFontsize)
plt.grid(True)
plt.tight_layout()


"""
Plot filterd acceleration
"""
plt.figure(figsize=commonFigsize)
plt.plot(timestamps, ax, label="EKF x accelerations", color="green", alpha=0.6)
plt.plot(timestamps, ay, label="EKF y accelerations", color="orange", alpha=0.6)
plt.xlabel("Time [s]", fontsize=labelFontsize)
plt.ylabel("Acceleration [m/s²]", fontsize=labelFontsize)
plt.xticks(fontsize=tickFontsize)
plt.yticks(fontsize=tickFontsize)
plt.title("Acceleration EKF", fontsize=titleFontsize)
plt.legend(fontsize=legendFontsize)
plt.grid(True)
plt.tight_layout()

plt.show()