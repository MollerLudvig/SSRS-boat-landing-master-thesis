import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import cartopy.crs as ccrs
import cartopy.feature as cfeature
from pyproj import Proj, CRS, Transformer
from cartopy.io.img_tiles import OSM
from kalman_OOSM import KalmanFilterXY


# Initialize UTM projection (choose zone based on starting lat/lon)
def get_utm_zone(lon):
    return int((lon + 180) / 6) + 1

def latlon_to_xy(lat, lon, lat0, lon0):
    """Convert lat/lon to local UTM XY coordinates."""
    utm_zone = get_utm_zone(lon0)
    utm_proj = Proj(proj="utm", zone=utm_zone, ellps="WGS84", south=lat0 < 0)
    x, y = utm_proj(lon, lat)
    x0, y0 = utm_proj(lon0, lat0)
    return x - x0, y - y0  # Relative XY

def xy_to_latlon(x, y, lat0, lon0):
    """Convert local XY back to lat/lon."""
    utm_zone = get_utm_zone(lon0)
    is_southern = lat0 < 0

    # Define CRS objects
    crs_utm = CRS.from_proj4(f"+proj=utm +zone={utm_zone} +ellps=WGS84 +{'south' if is_southern else ''}")
    crs_latlon = CRS.from_epsg(4326)  # WGS84

    transformer = Transformer.from_crs(crs_utm, crs_latlon, always_xy=True)

    x0, y0 = Proj(crs_utm)(lon0, lat0)
    lon, lat = transformer.transform(x + x0, y + y0)
    return lat, lon

# Load AIS data
# csv_file = "pythontest/Guidance/valo_3.csv"  
csv_file = "ylva_1.csv"
df = pd.read_csv(csv_file)

# Get initial reference position
lat0, lon0, v0, psi0, time0 = df.iloc[0]["Latitude"], df.iloc[0]["Longitude"], df.iloc[0]["Speed"], np.radians(df.iloc[0]["Heading"]), df.iloc[0]["TimestampUnix"]

lastTime = df.iloc[-1]["TimestampUnix"]

# Convert lat/lon to local XY
df["x"], df["y"] = zip(*df.apply(lambda row: latlon_to_xy(row["Latitude"], row["Longitude"], lat0, lon0), axis=1))

dfPlot = df.copy()


kf = KalmanFilterXY(vx = np.sin(psi0)*v0,vy = np.cos(psi0)*v0 , psi = psi0,timestamp = time0,  process_noise_variance=0.00001)


dt = 5  # Time step in seconds
t = time0 + dt
trajectory = []
velocity = []
while True:
    # Predict the state
    kf.predict(t, past_timestamp=t-dt)
    
    if df.empty:
            break

    # Update with AIS measurement
    while t > df.iloc[0]["TimestampUnix"]:
        x = df.iloc[0]["x"]
        y = df.iloc[0]["y"]
        yaw = np.radians(df.iloc[0]["Heading"])
        vx = np.sin(yaw)*df.iloc[0]["Speed"]
        vy = np.cos(yaw)*df.iloc[0]["Speed"]
        # print(f'speed: {df.iloc[0]["Speed"]}, yaw: {yaw}, vx: {vx}, vy: {vy}')
        z = np.array([[x], [y], [vx], [vy], [yaw]])
        kf.update_AIS(z , df.iloc[0]["TimestampUnix"])
        # kf.update_AIS(np.array([[df.iloc[0]["x"]], [df.iloc[0]["y"]], [vx], [vy], [np.radians(df.iloc[0]["Heading"])]]) , df.iloc[0]["TimestampUnix"])
        # kf.update_AIS(np.array([[df.iloc[0]["x"]], [df.iloc[0]["y"]], [np.sin(df.iloc[0]["Speed"])], [np.cos(np.radians(df.iloc[0]["Heading"]))*df.iloc[0]["Speed"]], [np.radians(df.iloc[0]["Heading"])]]) , df.iloc[0]["TimestampUnix"])
        df.drop(index=df.index[0], axis=0, inplace=True)
        if df.empty:
            break

    trajectory.append((kf.x[0], kf.x[1]))
    # print(f"vx: {kf.x[2]}, vy: {kf.x[3]}, yaw: {kf.x[4]}")
    velocity.append(kf.x[2,0])
    t += dt


# Use OpenStreetMap tile background
osm_tiles = OSM()
fig, ax = plt.subplots(figsize=(10, 6), subplot_kw={"projection": osm_tiles.crs})

# Set map extent
ax.set_extent([dfPlot["Longitude"].min(), dfPlot["Longitude"].max(),
               dfPlot["Latitude"].min(), dfPlot["Latitude"].max()], crs=ccrs.PlateCarree())

# Add OSM tiles (detailed coastline & islands)
ax.add_image(osm_tiles, 10)  # Higher zoom level = more detail

# Plot raw AIS data
ax.plot(dfPlot["Longitude"], dfPlot["Latitude"], 'ro-', markersize=3, transform=ccrs.PlateCarree(), label="Raw AIS Data")

# Convert filtered XY back to lat/lon for plotting
filtered_lats, filtered_lons = zip(*[xy_to_latlon(x[0], x[1], lat0, lon0) for x in trajectory])


# Plot Kalman Filtered trajectory
ax.plot(filtered_lons, filtered_lats, 'bo-', markersize=3, transform=ccrs.PlateCarree(), label="Kalman Filtered Path")

plt.legend()
plt.title("AIS Ship Trajectory with OSM Map Background")
plt.grid()
plt.show()