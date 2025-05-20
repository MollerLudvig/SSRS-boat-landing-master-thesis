import numpy as np
from dataclasses import dataclass, field
from typing import List
from variabels import VehicleData
from coordinate_conv import latlon_to_xy

@dataclass
class ColissionData:
    time: List[float] = field(default_factory=list)
    delta_time: List[float] = field(default_factory=list)
    delta_time_timestamp: List[float] = field(default_factory=list)
    collision: List[bool] = field(default_factory=list)
    distance: List[float] = field(default_factory=list)
    delta_x: List[float] = field(default_factory=list)
    delta_y: List[float] = field(default_factory=list)
    delta_z: List[float] = field(default_factory=list)
    delta_n: List[float] = field(default_factory=list)
    delta_e: List[float] = field(default_factory=list)
    delta_d: List[float] = field(default_factory=list)


def is_landed(collision_data: ColissionData, boatData: VehicleData, droneData: VehicleData, 
            landing_threshold: List[float] = [0, 0, 0], offset_transform: List[float] = [0, 0, 0],
            max_time_delta: float = 0.1):  # Add maximum allowed time difference

    # Get time ranges for both datasets
    boat_times = np.array(boatData.gps.time)
    drone_times = np.array(droneData.gps.time)
    
    # Find common time window
    start_time = max(drone_times[0] if len(drone_times) > 0 else 0, 
                     boat_times[0] if len(boat_times) > 0 else 0)
    end_time = min(drone_times[-1] if len(drone_times) > 0 else float('inf'), 
                   boat_times[-1] if len(boat_times) > 0 else float('inf'))
    
    # Skip processing if there's no overlap or if the dataset are empty
    if start_time >= end_time or len(drone_times) == 0 or len(boat_times) == 0:
        print("Warning: No overlapping time data or empty datasets")
        return
    
    # For logging/debugging
    min_delta = float('inf')
    max_delta = 0
    total_delta = 0
    points_processed = 0
    skipped_points = 0
    
    # Process drone data within the common time window
    # Leave margin at the end to avoid edge effects
    end_index = len(droneData.gps.time) - 5 if len(droneData.gps.time) > 5 else 0
    
    for time_ind, time_val in enumerate(droneData.gps.time[:end_index]):
        # Skip if outside common time window
        if time_val < start_time or time_val > end_time:
            continue
            
        # Skip if we've already processed this time
        if len(collision_data.time) > 0 and time_val <= collision_data.time[-1]:
            continue
        
        # Find closest boat data point
        diff_array = np.abs(np.array(boatData.gps.time) - time_val)
        closest_index = np.argmin(diff_array)
        time_diff = abs(time_val - boatData.gps.time[closest_index])
        
        # Skip if time difference is too large
        if time_diff > max_time_delta:
            skipped_points += 1
            collision_data.delta_time_timestamp.append(time_val)
            collision_data.delta_time.append(time_diff)
            continue
        
        # Update stats
        min_delta = min(min_delta, time_diff)
        max_delta = max(max_delta, time_diff)
        total_delta += time_diff
        points_processed += 1
            
        # Calculate position difference
        dn, de = latlon_to_xy(lat=droneData.gps.lat[time_ind],
                             lon=droneData.gps.lon[time_ind],
                             lat0=boatData.gps.lat[closest_index],
                             lon0=boatData.gps.lon[closest_index])
        
        # Get altitude difference (delta down): This is actuallt inverted so from drones perspective but thats for plotting reasons
        dd = droneData.gps.alt[time_ind] - boatData.gps.alt[closest_index]
        
        # NED to XYZ rotation around Z axis (apply boat's yaw)
        hdg_rad = -np.radians(boatData.gps.hdg[closest_index])
        dx = dn * np.cos(hdg_rad) - de * np.sin(hdg_rad)
        dy = dn * np.sin(hdg_rad) + de * np.cos(hdg_rad)
        dz = dd
        
        # Apply offset transform
        dx += offset_transform[0]
        dy += offset_transform[1]
        dz -= offset_transform[2]
        
        # Calculate distance
        distance = np.sqrt(dx**2 + dy**2 + dz**2)

        if np.abs(dx) < landing_threshold[0] and np.abs(dy) < landing_threshold[1] and np.abs(dz) < landing_threshold[2]:
            collision_data.collision.append(True)
            print(f"Collision detected at time {time_val:.4f}s: distance = {distance:.2f}m")
        else:
            collision_data.collision.append(False)
        
        # Record data
        collision_data.time.append(time_val)
        collision_data.delta_time.append(time_diff)
        collision_data.delta_time_timestamp.append(time_val)
        collision_data.distance.append(distance)
        collision_data.delta_x.append(dx)
        collision_data.delta_y.append(dy)
        collision_data.delta_z.append(dz)
        collision_data.delta_n.append(dn)
        collision_data.delta_e.append(de)
        collision_data.delta_d.append(dd)
    
    # Print time delta statistics
    if points_processed > 0:
        print(f"Time delta stats - Min: {min_delta:.4f}s, Max: {max_delta:.4f}s, " +
              f"Avg: {total_delta/points_processed:.4f}s, Skipped: {skipped_points} points")
    else:
        print("No data points were processed")