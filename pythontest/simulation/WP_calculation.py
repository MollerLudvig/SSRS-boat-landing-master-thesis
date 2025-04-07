import numpy as np

R = 6371000 # Earth radius

# Calculate distance between coordinates
def dist_between_coords(lat1, lon1, lat2, lon2):
    """Calculate the distance in meters between two points"""
    lat1_rad = np.deg2rad(lat1)
    lon1_rad = np.deg2rad(lon1)
    lat2_rad = np.deg2rad(lat2)
    lon2_rad = np.deg2rad(lon2)

    delta_lat = lat2_rad-lat1_rad
    delta_lon = lon2_rad-lon1_rad
    mean_lat = (lat1_rad+lat2_rad)/2
    d = np.sqrt((delta_lat*R)**2 + (delta_lon*R*np.cos(mean_lat))**2)
    return d

# Calculate coordinates a distance in heading
def calc_look_ahead_point(lat, lon, heading, look_ahead_dist):
    """Calculate a lookahead point from a coordinate given a heading
    Uses simple spherical coordinate and doesn't take earth curvature into account
    Since for the short distances we will be working with the earth can be seen as flat"""

    heading_rad = np.radians(heading)

    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    # Spherical geometry
    target_lat_rad = lat_rad + (look_ahead_dist / R) * np.cos(heading_rad)
    target_lon_rad = lon_rad + (look_ahead_dist / (R * np.cos(lat_rad))) * np.sin(heading_rad)

    target_lat = np.degrees(target_lat_rad)
    target_lon = np.degrees(target_lon_rad)

    return target_lat, target_lon

# Calculate distance to landing point
def calc_landing_point_dist_drone(d, drone_speed, boat_speed):
    """Calculate the distance that the drone needs to travel to the intercept point"""
    relative_speed = drone_speed - boat_speed
    time_to_catch_up = d/relative_speed
    waypoint_distance = drone_speed*time_to_catch_up
    return waypoint_distance

def calc_landing_point_dist_boat(d, drone_speed, boat_speed):
    """Calculate the distance that the boat needs to travel to the intercept point"""
    relative_speed = drone_speed - boat_speed
    time_to_catch_up = d/relative_speed
    waypoint_distance = boat_speed*time_to_catch_up
    return waypoint_distance

def calc_P2(drone_speed, boat_speed, altitude, glide_slope):
    """Calculate the distance behind the boat that P2 should be"""
    drone_intercept_distance = altitude/glide_slope
    time_to_intercept = drone_intercept_distance/drone_speed
    boat_intercept_distance = boat_speed*time_to_intercept
    P2_distance = drone_intercept_distance - boat_intercept_distance
    return P2_distance