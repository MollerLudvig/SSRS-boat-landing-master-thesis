import numpy as np
from dataclasses import dataclass, field
from typing import List
from variabels import VehicleData
from coordinate_conv import latlon_to_xy

@dataclass
class ColissionData:
    time: List[float] = field(default_factory=list)
    delta_time: List[float] = field(default_factory=list)
    collision: List[bool] = field(default_factory=list)
    distance: List[float] = field(default_factory=list)
    delta_x: List[float] = field(default_factory=list)
    delta_y: List[float] = field(default_factory=list)
    delta_z: List[float] = field(default_factory=list)


def is_landed(collision_data: ColissionData, boatData: VehicleData, droneData: VehicleData, 
            threshold: float = 2.0, offset_transform: List[float] = [0, 0, 0],
            max_time_delta: float = 0.1):  # Add maximum allowed time difference

    # Get time ranges for both datasets
    boat_times = np.array(boatData.simulation.time)
    drone_times = np.array(droneData.simulation.time)
    
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
    end_index = len(droneData.simulation.time) - 20 if len(droneData.simulation.time) > 20 else 0
    
    for time_ind, time_val in enumerate(droneData.simulation.time[:end_index]):
        # Skip if outside common time window
        if time_val < start_time or time_val > end_time:
            continue
            
        # Skip if we've already processed this time
        if len(collision_data.time) > 0 and time_val <= collision_data.time[-1]:
            continue
        
        # Find closest boat data point
        diff_array = np.abs(np.array(boatData.simulation.time) - time_val)
        closest_index = np.argmin(diff_array)
        time_diff = abs(time_val - boatData.simulation.time[closest_index])
        
        # Skip if time difference is too large
        if time_diff > max_time_delta:
            skipped_points += 1
            continue
        
        # Update stats
        min_delta = min(min_delta, time_diff)
        max_delta = max(max_delta, time_diff)
        total_delta += time_diff
        points_processed += 1
            
        # Calculate position difference
        dn, de = latlon_to_xy(lat=droneData.simulation.lat[time_ind],
                             lon=droneData.simulation.lon[time_ind],
                             lat0=boatData.simulation.lat[closest_index],
                             lon0=boatData.simulation.lon[closest_index])
        
        # Get altitude difference (delta down)
        dd = boatData.simulation.alt[closest_index] - droneData.simulation.alt[time_ind]
        
        # NED to XYZ rotation around Z axis (apply boat's yaw)
        yaw_rad = np.radians(boatData.simulation.yaw[closest_index])
        dx = dn * np.cos(yaw_rad) - de * np.sin(yaw_rad)
        dy = dn * np.sin(yaw_rad) + de * np.cos(yaw_rad)
        dz = dd
        
        # Apply offset transform
        dx += offset_transform[0]
        dy += offset_transform[1]
        dz += offset_transform[2]
        
        # Calculate distance
        distance = np.sqrt(dx**2 + dy**2 + dz**2)
        
        # Record data
        collision_data.time.append(time_val)
        collision_data.delta_time.append(time_diff)
        collision_data.collision.append(distance < threshold)
        collision_data.distance.append(distance)
        collision_data.delta_x.append(dx)
        collision_data.delta_y.append(dy)
        collision_data.delta_z.append(dz)
        
        if collision_data.collision[-1]:
            print(f"Collision detected at time {time_val:.4f}s: distance = {distance:.2f}m")
    
    # Print time delta statistics
    if points_processed > 0:
        print(f"Time delta stats - Min: {min_delta:.4f}s, Max: {max_delta:.4f}s, " +
              f"Avg: {total_delta/points_processed:.4f}s, Skipped: {skipped_points} points")
    else:
        print("No data points were processed")