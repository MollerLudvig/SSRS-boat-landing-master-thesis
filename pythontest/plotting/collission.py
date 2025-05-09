import numpy as np
from dataclasses import dataclass, field
from typing import List
from variabels import VehicleData
from coordinate_conv import latlon_to_ned

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


def is_landed(collision_data: ColissionData, boatData: VehicleData, droneData: VehicleData, threshold: float = 2.0, offset_transform: List[float] = [0, 0, 0]):
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
    
    # Check if required attributes are available
    required_attrs = ['lat', 'lon', 'alt', 'yaw']
    for attr in required_attrs:
        if (not hasattr(boatData.simulation, attr) or not hasattr(droneData.simulation, attr) or
            getattr(boatData.simulation, attr) is None or getattr(droneData.simulation, attr) is None):
            print(f"Warning: Missing required attribute {attr} for landing detection")
            return  # Exit function early

    # Get what index last checked time is now at
    # Has to be done since old values are thrown away
    if len(collision_data.time) > 0:
        last_timecheck = collision_data.time[-1]
        if last_timecheck < droneData.simulation.time[0]:
            collision_data.last_indexcheck_d = 0
            collision_data.last_indexcheck_b = 0
        else:
            collision_data.last_indexcheck_b =-1
            collision_data.last_indexcheck_d =-1
            while last_timecheck <= (boatData.simulation.time[collision_data.last_indexcheck_b]):
               collision_data.last_indexcheck_b -=1
            while last_timecheck <= (droneData.simulation.time[collision_data.last_indexcheck_d] - 0.1): #add some margin on drone time since it is not crucial
                collision_data.last_indexcheck_d -=1


    closest_index = 0
    for time_ind, time_val in enumerate(boatData.simulation.time[collision_data.last_indexcheck_b:]):
        # Ceck distance to every unchecked point 
        diff_array = np.abs(np.array(droneData.simulation.time[collision_data.last_indexcheck_d:]) - time_val)
        
        # If noting in diff array we cooked
        if len(diff_array) == 0:
            continue
            
        # Get the index of the closest point in droneData
        local_closest_index = np.argmin(diff_array)
        closest_index = local_closest_index + collision_data.last_indexcheck_d
        
        # Get delta North and East of 
        dn, de = latlon_to_ned(lat = droneData.simulation.lat[closest_index],
                                    lon = droneData.simulation.lon[closest_index],
                                    lat0 = boatData.simulation.lat[time_ind],
                                    lon0 = boatData.simulation.lon[time_ind])
        
        # Get delta Down
        dd = boatData.simulation.alt[time_ind] - droneData.simulation.alt[closest_index]

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
    
    # BIGG UPP THAT INDEXX BOIE but chill if we have no new data
    if closest_index:
        collision_data.last_indexcheck_d = closest_index
        collision_data.last_indexcheck_b = len(boatData.simulation.time)

    return collision_data