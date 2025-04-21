import numpy as np
import pandas as pd

# import matplotlib
# matplotlib.use("Agg")  # Use non-interactive backend
import matplotlib.pyplot as plt

import cartopy.crs as ccrs
import cartopy.feature as cfeature
from cartopy.io.img_tiles import OSM
from kalman_OOSM import KalmanFilterXY
from coordinate_conv import latlon_to_xy, xy_to_latlon

# Load AIS data
# csv_file = "pythontest/Guidance/valo_3.csv"  
# csv_file = "pythontest/simulation/Guidance/data_short.csv"
# csv_file = "data_short_short.csv"
csv_file = "data_short.csv"
# csv_file = "ssrs-josephine_1.csv"

df = pd.read_csv(csv_file)

# Get initial reference position
lat0, lon0, v0, psi0, time0 = df.iloc[0]["lat"], df.iloc[0]["lon"], df.iloc[0]["speed[m/s]"], np.radians(df.iloc[0]["heading"]), df.iloc[0]["timestamp_unix"]

lastTime = df.iloc[-1]["timestamp_unix"]

# Convert lat/lon to local XY
df["x"], df["y"] = zip(*df.apply(lambda row: latlon_to_xy(row["lat"], row["lon"], lat0, lon0), axis=1))

dfPlot = df.copy()


print(dfPlot[["x", "y", "speed[m/s]"]].head(10))


kf = KalmanFilterXY(v = v0, heading=psi0, init_lat=lat0, init_lon=lon0, timestamp = time0,  process_noise_variance=0.001,)

t = time0
dt = 1  # Time step in seconds
t += dt
trajectory = []
velocity = []
while True:
    # Predict the state
    kf.predict(t, past_timestamp=t-dt)
    
    if df.empty:
            break

    # Update with AIS measurement
    while t > df.iloc[0]["timestamp_unix"]:
        # kf.update_AIS(np.array([[df.iloc[0]["x"]], [df.iloc[0]["y"]], [df.iloc[0]["speed[m/s]"]], [np.radians(df.iloc[0]["course"])]]) , df.iloc[0]["timestamp_unix"])
        kf.update_w_latlon(np.array([[df.iloc[0]["lat"]], [df.iloc[0]["lon"]], [df.iloc[0]["speed[m/s]"]], [df.iloc[0]["heading"]]]) , df.iloc[0]["timestamp_unix"])
        df.drop(index=df.index[0], axis=0, inplace=True)
        if df.empty:
            break

    trajectory.append((kf.x[0], kf.x[1]))
    velocity.append(kf.x[2,0])
    t += dt

# Convert filtered XY back to lat/lon for plotting
filtered_lats, filtered_lons = zip(*[xy_to_latlon(x[0], x[1], lat0, lon0) for x in trajectory])

# print (f"Filtered trajectory: {filtered_lats}, {filtered_lons}")


# Use OpenStreetMap tile background
osm_tiles = OSM()
fig, ax = plt.subplots(figsize=(10, 6), subplot_kw={"projection": osm_tiles.crs})

# Set map extent
ax.set_extent([min([dfPlot["lon"].min(), min(filtered_lons)]), max([dfPlot["lon"].max(), max(filtered_lons)]),
                min([dfPlot["lat"].min(), min(filtered_lats)]), max([dfPlot["lat"].max(), max(filtered_lats)])], crs=ccrs.PlateCarree())

# Add OSM tiles (detailed coastline & islands)
ax.add_image(osm_tiles, 18)  # Higher zoom level = more detail

# Plot raw AIS data
ax.plot(dfPlot["lon"], dfPlot["lat"], 'ro-', markersize=1, transform=ccrs.PlateCarree(), label="Raw AIS Data")




# Plot Kalman Filtered trajectory
ax.plot(filtered_lons, filtered_lats, 'bo-', markersize=1, transform=ccrs.PlateCarree(), label="Kalman Filtered Path")

plt.legend()
plt.title("AIS Ship Trajectory with OSM Map Background")
plt.grid()
plt.show()
# plt.savefig("filtered_trajectory.png")  # Save the plot to file
