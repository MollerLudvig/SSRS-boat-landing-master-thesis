import numpy as np
from dataclasses import dataclass, field
from typing import List
from variabels import VehicleData
from coordinate_conv import latlon_to_ned, latlon_to_xy

@dataclass
class ColissionData:
    time: List[float] = field(default_factory=list)
    delta_time: List[float] = field(default_factory=list)
    collision: List[bool] = field(default_factory=list)
    distance: List[float] = field(default_factory=list)
    delta_x: List[float] = field(default_factory=list)
    delta_y: List[float] = field(default_factory=list)
    delta_z: List[float] = field(default_factory=list)

    last_indexcheck_b: int = 0
    last_indexcheck_d: int = 0


def bad(collision_data: ColissionData, boatData: VehicleData, droneData: VehicleData, 
            threshold: float = 2.0, offset_transform: List[float] = [0, 0, 0]):    
    """
    Check if drone has landed (bingbong)
    """

    # Check if data is available
    if (not hasattr(boatData, 'simulation') or not hasattr(droneData, 'simulation') or
        not hasattr(boatData.simulation, 'time') or not hasattr(droneData.simulation, 'time') or
        boatData.simulation.time is None or droneData.simulation.time is None or
        len(boatData.simulation.time) == 0 or len(droneData.simulation.time) == 0):
        print("Warning: Vehicle data not available yet for landing detection")
        return  # Exit function early
    
    # # Ensure last_indexcheck values don't exceed array bounds
    if collision_data.last_indexcheck_b >= len(boatData.simulation.time):
        collision_data.last_indexcheck_b = len(boatData.simulation.time) - 1
        
    if collision_data.last_indexcheck_d >= len(droneData.simulation.time):
        collision_data.last_indexcheck_d = len(droneData.simulation.time) - 1
    

    # Get what index last checked time is now at
    # Has to be done since old values are thrown away
    if len(collision_data.time) > 0:
        last_timecheck = collision_data.time[-1]
        if last_timecheck < droneData.simulation.time[0]:
            collision_data.last_indexcheck_d = 0
            collision_data.last_indexcheck_b = 0
        else:
            # make sure no out of range inexing
            if collision_data.last_indexcheck_b >= len(boatData.simulation.time) and collision_data.last_indexcheck_b != 0:
                collision_data.last_indexcheck_b = len(boatData.simulation.time) - 1
            if collision_data.last_indexcheck_d >= len(droneData.simulation.time) and collision_data.last_indexcheck_d != 0:
                collision_data.last_indexcheck_d = len(droneData.simulation.time) - 1

            # Check if the last time check is before the current time
            while last_timecheck < (boatData.simulation.time[collision_data.last_indexcheck_b]):
               if collision_data.last_indexcheck_b == 0:
                    break
               collision_data.last_indexcheck_b -=1
            while last_timecheck < (droneData.simulation.time[collision_data.last_indexcheck_d] - 0.1): #add some margin on drone time since it is not crucial
                if collision_data.last_indexcheck_d == 0:
                    break
                collision_data.last_indexcheck_d -=1



    closest_index = 0
    for time_ind, time_val in enumerate(boatData.simulation.time[collision_data.last_indexcheck_b:]):
    # itterate over the last 20 timesteps
    # for time_ind, time_val in enumerate(boatData.simulation.time):
        # Ceck distance to every unchecked point 
        diff_array = np.abs(np.array(droneData.simulation.time) - time_val)
        
        # If noting in diff array we cooked
        if len(diff_array) == 0:
            continue
            
        # Get the index of the closest point in droneData
        local_closest_index = np.argmin(diff_array)
        # closest_index = local_closest_index + collision_data.last_indexcheck_d
        closest_index = local_closest_index
        
        # Get delta North and East of 
        dn, de = latlon_to_xy(lat = droneData.simulation.lat[closest_index],
                                    lon = droneData.simulation.lon[closest_index],
                                    lat0 = boatData.simulation.lat[time_ind],
                                    lon0 = boatData.simulation.lon[time_ind])
        
        # Get delta Down
        dd = boatData.simulation.alt[time_ind] - droneData.simulation.alt[closest_index]
        print(f"alt_boat: {boatData.simulation.alt[time_ind]}, alt_drone: {droneData.simulation.alt[closest_index]}, dd: {dd}")

        # NED to XYZ rotation arond Z axis
        yaw_rad = np.radians(boatData.simulation.yaw[time_ind])
        dx = dn * np.cos(yaw_rad) - de * np.sin(yaw_rad)
        dy = dn * np.sin(yaw_rad) + de * np.cos(yaw_rad)
        dz = dd

        # Apply offset transform
        dx += offset_transform[0]
        dy += offset_transform[1]
        dz += offset_transform[2]
        
        # Calculate distance
        distance = np.sqrt(dx**2 + dy**2 + dz**2)


        # Append information for timestamp
        collision_data.time.append(time_val)
        collision_data.delta_time.append(abs(time_val - droneData.simulation.time[closest_index]))
        collision_data.collision.append(distance < threshold)
        collision_data.distance.append(distance)
        collision_data.delta_x.append(dn)
        collision_data.delta_y.append(de)
        collision_data.delta_z.append(dz)

        if collision_data.collision[-1]:
            print(f"Collision detected at time {collision_data.time[-1]:.2f} s: distance = {collision_data.distance[-1]:.2f} m")
    
    # BIGG UPP THAT INDEXX BOIE but chill if we have no new data
    collision_data.last_indexcheck_b = len(boatData.simulation.time) - 1
    if closest_index:
            collision_data.last_indexcheck_d = closest_index


def second_try(collision_data: ColissionData, boatData: VehicleData, droneData: VehicleData, 
            threshold: float = 2.0, offset_transform: List[float] = [0, 0, 0]):
    
    # Itterate over boatData
    itterator = 0

    len_boat = len(boatData.simulation.time)

    for time_ind, time_val in enumerate(boatData.simulation.time):
        # skip the last 10 points
        if time_ind > len_boat - 20:
            break

        # print(f"Time: {time_val:.2f} s")

        # Skip if the time is less than or equal to the last recorded time
        if len(collision_data.time) > 0 and time_val <= collision_data.time[-1]:
            # print(f"Breaking time {time_val} s: already checked")
            continue
        

        # Get the index of the closest point in droneData
        diff_array = np.abs(np.array(droneData.simulation.time) - time_val)
        closest_index = np.argmin(diff_array)

        # Get delta North and East of 
        dn, de = latlon_to_xy(lat = droneData.simulation.lat[closest_index],
                                    lon = droneData.simulation.lon[closest_index],
                                    lat0 = boatData.simulation.lat[time_ind],
                                    lon0 = boatData.simulation.lon[time_ind])
        
        # Get delta Down
        dd = boatData.simulation.alt[time_ind] - droneData.simulation.alt[closest_index]
        # print(f"alt_boat: {boatData.simulation.alt[time_ind]}, alt_drone: {droneData.simulation.alt[closest_index]}, dd: {dd}")

        # NED to XYZ rotation arond Z axis
        yaw_rad = np.radians(boatData.simulation.yaw[time_ind])
        dx = dn * np.cos(yaw_rad) - de * np.sin(yaw_rad)
        dy = dn * np.sin(yaw_rad) + de * np.cos(yaw_rad)
        dz = dd

        # Apply offset transform
        dx += offset_transform[0]
        dy += offset_transform[1]
        dz += offset_transform[2]
        
        # Calculate distance
        distance = np.sqrt(dx**2 + dy**2 + dz**2)


        # Append information for timestamp
        collision_data.time.append(time_val)
        collision_data.delta_time.append(abs(time_val - droneData.simulation.time[closest_index]))
        collision_data.collision.append(distance < threshold)
        collision_data.distance.append(distance)
        collision_data.delta_x.append(dx)
        collision_data.delta_y.append(dy)
        collision_data.delta_z.append(dz)

        if collision_data.collision[-1]:
            print(f"Collision detected at time {collision_data.time[-1]:.2f} s: distance = {collision_data.distance[-1]:.2f} m")


def is_landed(collision_data: ColissionData, boatData: VehicleData, droneData: VehicleData, 
            threshold: float = 2.0, offset_transform: List[float] = [0, 0, 0],
            max_time_delta: float = 0.1):  # Add maximum allowed time difference
    """
    Detect if the drone has landed on the boat by checking the distance between them.
    
    Args:
        collision_data: Data structure to store collision information
        boatData: Boat telemetry data
        droneData: Drone telemetry data
        threshold: Distance threshold for considering landed (meters)
        offset_transform: Offset to apply to the calculated position [x, y, z]
        max_time_delta: Maximum allowed time difference between boat and drone data points (seconds)
    """
    # Get time ranges for both datasets
    boat_times = np.array(boatData.simulation.time)
    drone_times = np.array(droneData.simulation.time)
    
    # Find common time window
    start_time = max(boat_times[0] if len(boat_times) > 0 else 0, 
                     drone_times[0] if len(drone_times) > 0 else 0)
    end_time = min(boat_times[-1] if len(boat_times) > 0 else float('inf'), 
                   drone_times[-1] if len(drone_times) > 0 else float('inf'))
    
    # Skip processing if there's no overlap or if either dataset is empty
    if start_time >= end_time or len(boat_times) == 0 or len(drone_times) == 0:
        print("Warning: No overlapping time data or empty datasets")
        return
    
    # For logging/debugging
    min_delta = float('inf')
    max_delta = 0
    total_delta = 0
    points_processed = 0
    skipped_points = 0
    
    # Process boat data within the common time window
    # Leave margin at the end to avoid edge effects
    end_index = len(boatData.simulation.time) - 20 if len(boatData.simulation.time) > 20 else 0
    
    for time_ind, time_val in enumerate(boatData.simulation.time[:end_index]):
        # Skip if outside common time window
        if time_val < start_time or time_val > end_time:
            continue
            
        # Skip if we've already processed this time
        if len(collision_data.time) > 0 and time_val <= collision_data.time[-1]:
            continue
        
        # Find closest drone data point
        diff_array = np.abs(np.array(droneData.simulation.time) - time_val)
        closest_index = np.argmin(diff_array)
        time_diff = abs(time_val - droneData.simulation.time[closest_index])
        
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
        dn, de = latlon_to_xy(lat=droneData.simulation.lat[closest_index],
                             lon=droneData.simulation.lon[closest_index],
                             lat0=boatData.simulation.lat[time_ind],
                             lon0=boatData.simulation.lon[time_ind])
        
        # Get altitude difference (delta down)
        dd = boatData.simulation.alt[time_ind] - droneData.simulation.alt[closest_index]
        
        # NED to XYZ rotation around Z axis (apply boat's yaw)
        yaw_rad = np.radians(boatData.simulation.yaw[time_ind])
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
            print(f"Collision detected at time {time_val:.2f}s: distance = {distance:.2f}m")
    
    # Print time delta statistics
    if points_processed > 0:
        print(f"Time delta stats - Min: {min_delta:.4f}s, Max: {max_delta:.4f}s, " +
              f"Avg: {total_delta/points_processed:.4f}s, Skipped: {skipped_points} points")
    else:
        print("No data points were processed")