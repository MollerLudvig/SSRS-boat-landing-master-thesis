from pyproj import Proj, CRS, Transformer

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