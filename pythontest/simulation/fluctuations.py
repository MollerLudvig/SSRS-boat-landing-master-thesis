import numpy as np

def boat_movement(flag, magnitude, desired_boat_direction):
    boat_heading_fluct = np.random.normal(0, magnitude)
    noisy_boat_direction = desired_boat_direction + boat_heading_fluct

    boat_direction = desired_boat_direction
    if flag:
        boat_direction = noisy_boat_direction

    return boat_direction

def boat_altitude(flag, magnitude, base_altitude, i):
    boat_alt_fluct = magnitude*np.sin(i*0.2) + np.random.normal(0, magnitude/3)
    noisy_boat_alt = base_altitude + boat_alt_fluct

    boat_alt = base_altitude
    if flag:
        boat_alt = noisy_boat_alt
        
    return boat_alt

def drone_throttle(flag, magnitude, base_throttle):
    drone_throttle_fluct = np.random.normal(0, magnitude)
    noisy_throttle = base_throttle + drone_throttle_fluct

    output_throttle = base_throttle
    if flag:
        output_throttle = noisy_throttle
    
    return output_throttle