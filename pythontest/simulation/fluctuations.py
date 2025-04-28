import numpy as np

def boat_movement(flag, magnitude):
    desired_boat_direction = 0
    boat_heading_fluct = magnitude*2*(np.random.rand()-0.5)
    noisy_boat_direction = desired_boat_direction + boat_heading_fluct

    boat_direction = desired_boat_direction
    if flag:
        boat_direction = noisy_boat_direction

    return boat_direction