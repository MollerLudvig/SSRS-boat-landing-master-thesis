import numpy as np
import pandas as pd

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
# csv_file = "simulation/Guidance/data_short.csv"
# csv_file = "data_short_short.csv"
csv_file = "data_short2.csv"
# csv_file = "data_short_OOSM.csv"
# csv_file = "ssrs-josephine_1.csv"

df = pd.read_csv(csv_file)

# Get initial reference position
lat0, lon0, v0, psi0, time0 = df.iloc[0]["lat"], df.iloc[0]["lon"], df.iloc[0]["speed[m/s]"], df.iloc[0]["course"], df.iloc[0]["timestamp_unix"]

lastTime = df.iloc[-1]["timestamp_unix"]

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

t = time0
dt = 1  # Time step in seconds
t += dt
trajectory = []
lats = []
lons = []
u_vel = []
v_vel = []
velocity = []
heading = []
velocity_heading = []
yawrate = []
timestamps = []
delta_time = []

while True:
    # Update with all AIS measurements up to current time t
    while not df.empty and t >= df.iloc[0]["timestamp_unix"]:

        heading_rad = np.radians(df.iloc[0]["heading"])
        course_rad = np.radians(df.iloc[0]["course"])

        delta_r = course_rad - heading_rad
        # Normalize delta_r to be within -pi to pi
        delta_r = (delta_r + np.pi) % (2*np.pi) - np.pi

        u = df.iloc[0]["speed[m/s]"] * np.cos(delta_r)
        v = df.iloc[0]["speed[m/s]"] * np.sin(delta_r)

        z = np.array([
            [df.iloc[0]["lat"]],
            [df.iloc[0]["lon"]],
            [df.iloc[0]["heading"]],
            [u],
            [v]
        ])

        measurment_time = df.iloc[0]["timestamp_unix"]
        kf.update_w_latlon(z, measurment_time)

        df.drop(index=df.index[0], inplace=True)

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
    velocity_heading.append(np.rad2deg(np.arctan2(v, u)))
    heading.append(kf.x[2, 0])
    yawrate.append(kf.x[5, 0])
    timestamps.append(t)

    # print(f"t: {t}, lat: {kf.lat}, lon: {kf.lon}, u: {u}, v: {v}, heading: {np.rad2deg(kf.x[2, 0])}, yawrate: {kf.x[5, 0]}")

    if t > lastTime:
        break
    t += dt


# Convert filtered XY back to lat/lon for plotting
filtered_lats, filtered_lons = zip(*[ned_to_latlon(x[0], x[1], lat0, lon0) for x in trajectory])
# filtered_lats = lats
# filtered_lons = lons

# print (f"Filtered trajectory: {filtered_lats}, {filtered_lons}")


# !----- Plotting -----!
# measuredHeadings = dfPlot["heading"]  # in degrees

# # Check if dfPlot is empty or contains only NaNs in lat/lon
# if dfPlot.empty or dfPlot["lat"].isnull().all() or dfPlot["lon"].isnull().all():
#     raise ValueError("dfPlot is empty or contains only NaNs in lat/lon.")


# # Use OpenStreetMap tile background
# osm_tiles = OSM()
# fig, ax = plt.subplots(figsize=(10, 6), subplot_kw={"projection": osm_tiles.crs})

# # Set map extent
# ax.set_extent([min([dfPlot["lon"].min(), min(filtered_lons)]), max([dfPlot["lon"].max(), max(filtered_lons)]),
#                 min([dfPlot["lat"].min(), min(filtered_lats)]), max([dfPlot["lat"].max(), max(filtered_lats)])], crs=ccrs.PlateCarree())

# # Add OSM tiles (detailed coastline & islands)
# ax.add_image(osm_tiles, 10)  # Higher zoom level = more detail

# # Plot raw AIS data
# ax.plot(dfPlot["lon"].to_numpy(), dfPlot["lat"].to_numpy(), 'ro-', markersize=3, transform=ccrs.PlateCarree(), label="Raw AIS Data")
# plt.plot(dfPlot["timestamp_unix"].to_numpy(), dfPlot["speed[m/s]"].to_numpy(), label="Measured Velocity", color="red", alpha=0.6)
# plt.plot(dfPlot["timestamp_unix"].to_numpy(), measuredHeadings.to_numpy(), label="Measured Heading", color="red", alpha=0.6)


# # Plot Kalman Filtered trajectory
# ax.plot(filtered_lons, filtered_lats, 'bo-', markersize=3, transform=ccrs.PlateCarree(), label="Kalman Filtered Path")

# plt.legend()
# plt.title("AIS Ship Trajectory with OSM Map Background")
# plt.grid()
# plt.show()
# plt.savefig("filtered_trajectory.png")  # Save the plot to file


"""
Plot latlons
"""
plt.figure(figsize=(10, 6))
plt.plot(dfPlot["lon"].to_numpy(), dfPlot["lat"].to_numpy(), 'ro-', markersize=3, label="Raw AIS Data")
plt.plot(filtered_lons, filtered_lats, 'bo-', markersize=3, label="Kalman Filtered Path")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.title("Kalman Filtered Path vs Raw AIS Data")
plt.legend()
plt.grid()
plt.tight_layout()


"""
Plot filtered trajectory
"""
plt.figure(figsize=(10, 4))
plt.plot(timestamps, velocity, label="Filtered Velocity", color="blue")
plt.plot(dfPlot["timestamp_unix"].to_numpy(), dfPlot["speed[m/s]"].to_numpy(), label="Measured Velocity", color="red", alpha=0.6)
plt.plot(timestamps, u_vel, label="Filtered u velocity", color="green", alpha=0.6)
plt.plot(timestamps, v_vel, label="Filtered v velocity", color="orange", alpha=0.6)
plt.xlabel("Time [s]")
plt.ylabel("Velocity [m/s]")
plt.title("Velocity: Measured vs Kalman Filtered")
plt.legend()
plt.grid(True)
plt.tight_layout()




"""
Plot filtered heading
"""

plt.figure(figsize=(10, 4))
plt.plot(timestamps, heading, label="Filtered Heading", color="blue")
plt.plot(dfPlot["timestamp_unix"].to_numpy(), dfPlot["heading"].to_numpy(), label="Measured Heading", color="red", alpha=0.6)
plt.xlabel("Time [s]")
plt.ylabel("Heading [deg]")
plt.title("Heading: Measured vs Kalman Filtered")
plt.legend()
plt.grid(True)
plt.tight_layout()


"""
Plot timestams
"""
plt.figure(figsize=(10, 4))
plt.plot(timestamps, label="Timestamps", color="blue")
plt.xlabel("Time [s]")
plt.ylabel("Timestamps [s]")
plt.title("Timestamps")
plt.legend()
plt.grid(True)
plt.tight_layout()


"""
Plot delta time
"""
plt.figure(figsize=(10, 4))
plt.plot(timestamps, delta_time, label="Delta Time", color="blue")
plt.xlabel("Time [s]")
plt.ylabel("Delta Time [s]")
plt.title("Delta Time")
plt.legend()
plt.grid(True)
plt.tight_layout()



"""
Plot filtered yawrate
"""
plt.figure(figsize=(10, 4))
plt.plot(timestamps, yawrate, label="Filtered Yawrate", color="blue")
plt.xlabel("Time [s]")
plt.ylabel("Yawrate [rad/s]")
plt.title("Yawrate: Kalman Filtered")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()