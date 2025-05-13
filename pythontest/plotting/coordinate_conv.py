from pyproj import CRS, Transformer
import numpy as np

def get_utm_zone(lon):
    return int((lon + 180) / 6) + 1

def latlon_to_xy_vectors(lat, lon, lat0, lon0):
    """Convert lat/lon to local UTM XY coordinates relative to (lat0, lon0)."""
    utm_zone = get_utm_zone(lon0)
    is_southern = lat0 < 0

    crs_latlon = CRS.from_epsg(4326)  # WGS84
    crs_utm = CRS.from_proj4(f"+proj=utm +zone={utm_zone} +ellps=WGS84 {'+south' if is_southern else ''}")
    transformer = Transformer.from_crs(crs_latlon, crs_utm, always_xy=True)

    # Convert to numpy arrays for vectorized subtraction
    lon = np.asarray(lon)
    lat = np.asarray(lat)

    x, y = transformer.transform(lon, lat)
    x0, y0 = transformer.transform(lon0, lat0)

    return x - x0, y - y0

def latlon_to_xy(lat, lon, lat0, lon0):
    """Convert lat/lon to local UTM XY coordinates relative to (lat0, lon0)."""
    utm_zone = get_utm_zone(lon0)
    is_southern = lat0 < 0

    crs_latlon = CRS.from_epsg(4326)  # WGS84
    crs_utm = CRS.from_proj4(f"+proj=utm +zone={utm_zone} +ellps=WGS84 {'+south' if is_southern else ''}")

    transformer = Transformer.from_crs(crs_latlon, crs_utm, always_xy=True)

    y, x = transformer.transform(lon, lat)
    y0, x0 = transformer.transform(lon0, lat0)
    return x - x0, y - y0

def xy_to_latlon(x, y, lat0, lon0):
    """Convert local XY back to lat/lon relative to (lat0, lon0)."""
    utm_zone = get_utm_zone(lon0)
    is_southern = lat0 < 0

    crs_latlon = CRS.from_epsg(4326)
    crs_utm = CRS.from_proj4(f"+proj=utm +zone={utm_zone} +ellps=WGS84 {'+south' if is_southern else ''}")

    transformer_to_latlon = Transformer.from_crs(crs_utm, crs_latlon, always_xy=True)
    transformer_to_utm = Transformer.from_crs(crs_latlon, crs_utm, always_xy=True)

    y0, x0 = transformer_to_utm.transform(lon0, lat0)
    lon, lat = transformer_to_latlon.transform(y + y0, x + x0)
    return lat, lon





"""
local tangent transformation
"""

# Define the two CRSs
CRS_WGS84 = CRS.from_epsg(4326)  # lat/lon on WGS84 ellipsoid

def make_ltp_crs(lat0, lon0):
    """
    Create a Local Tangent Plane CRS (Azimuthal Equidistant) centered at (lat0, lon0).
    """
    # +proj=aeqd: Azimuthal Equidistant
    # +lat_0, +lon_0: center of the projection
    # +units=m, +ellps=WGS84: meters on WGS84 ellipsoid
    proj4 = (
        f"+proj=aeqd +lat_0={lat0} +lon_0={lon0} "
        f"+units=m +ellps=WGS84"
    )
    return CRS.from_proj4(proj4)

def latlon_to_ned(lat, lon, lat0, lon0):
    """
    Convert geographic coordinates to local NED (north, east) coordinates
    on a Local Tangent Plane centered at (lat0, lon0).
    
    Returns:
        north (float): meters north of the origin
        east  (float): meters east of the origin
    """
    crs_ltp = make_ltp_crs(lat0, lon0)
    transformer = Transformer.from_crs(CRS_WGS84, crs_ltp, always_xy=True)
    
    # Transformer returns (easting, northing)
    easting, northing = transformer.transform(lon, lat)
    
    # NED convention: x = north, y = east
    north = northing
    east  = easting
    return north, east

def ned_to_latlon(north, east, lat0, lon0):
    """
    Convert local NED (north, east) coordinates on a Local Tangent Plane
    back to geographic (lat, lon).
    
    Returns:
        lat (float), lon (float)
    """
    crs_ltp = make_ltp_crs(lat0, lon0)
    transformer = Transformer.from_crs(crs_ltp, CRS_WGS84, always_xy=True)
    
    # Inverse transform takes (easting, northing)
    lon, lat = transformer.transform(east, north)
    return lat, lon