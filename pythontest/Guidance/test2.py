import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from kalman_OOSM import KalmanFilterXY

# Load AIS CSV data
csv_file = "pythontest/Guidance/valo_3.csv"  # Update with the actual file path
df = pd.read_csv(csv_file)

# Initialize Kalman filter with the first AIS data point
initial_lat = df.iloc[0]["Latitude"]
initial_lon = df.iloc[0]["Longitude"]
kf = KalmanFilterXY(timestamp=df.iloc[0]["TimestampUnix"], v=df.iloc[0]["Speed"], psi=np.radians(df.iloc[0]["Heading"]))

# Extract timestamps
timestamps = df["TimestampUnix"].to_numpy()
start_time = timestamps[0]
end_time = timestamps[-1]

# Prediction and update loop
predicted_trajectory = []
measurement_index = 0

current_time = start_time
dt=5
while current_time <= end_time:
    kf.predict(dt)
    
    # Check if we have a new measurement at this time
    if measurement_index < len(timestamps) and timestamps[measurement_index] <= current_time:
        row = df.iloc[measurement_index]
        measurement = np.array([[row["Latitude"]], 
                                [row["Longitude"]], 
                                [row["Speed"]], 
                                [0],  # Acceleration is unknown
                                [np.radians(row["Heading"])]])
        print(f"Processing measurement: {measurement.T} at timestamp: {timestamps[measurement_index]}")
        kf.update_AIS(measurement, timestamps[measurement_index])
        measurement_index += 1  # Move to next measurement
    
    # Store predicted state for plotting
    predicted_trajectory.append((kf.x[0, 0], kf.x[1, 0]))
    
    current_time += dt  # Advance by 1 second

# Extract filtered trajectory
lats, lons = zip(*predicted_trajectory)

# Plot the results
plt.figure(figsize=(10, 6))
plt.plot(df["Longitude"].to_numpy(), df["Latitude"].to_numpy(), 'ro-', markersize=3, label="Raw AIS Data")
plt.plot(lons, lats, 'bo-', markersize=3, label="Kalman Filtered (1Hz Prediction)")
plt.xlabel("Longitude")
plt.ylabel("Latitude")
plt.legend()
plt.title("AIS Ship Trajectory with 1Hz Kalman Filtering")
plt.grid()
plt.show()