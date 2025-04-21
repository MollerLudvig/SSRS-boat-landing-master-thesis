from pyproj import CRS, Transformer

def get_utm_zone(lon):
    return int((lon + 180) / 6) + 1

def latlon_to_xy(lat, lon, lat0, lon0):
    """Convert lat/lon to local UTM XY coordinates relative to (lat0, lon0)."""
    utm_zone = get_utm_zone(lon0)
    is_southern = lat0 < 0

    crs_latlon = CRS.from_epsg(4326)  # WGS84
    crs_utm = CRS.from_proj4(f"+proj=utm +zone={utm_zone} +ellps=WGS84 {'+south' if is_southern else ''}")

    transformer = Transformer.from_crs(crs_latlon, crs_utm, always_xy=True)

    x, y = transformer.transform(lon, lat)
    x0, y0 = transformer.transform(lon0, lat0)
    return x - x0, y - y0

def xy_to_latlon(x, y, lat0, lon0):
    """Convert local XY back to lat/lon relative to (lat0, lon0)."""
    utm_zone = get_utm_zone(lon0)
    is_southern = lat0 < 0

    crs_latlon = CRS.from_epsg(4326)
    crs_utm = CRS.from_proj4(f"+proj=utm +zone={utm_zone} +ellps=WGS84 {'+south' if is_southern else ''}")

    transformer_to_latlon = Transformer.from_crs(crs_utm, crs_latlon, always_xy=True)
    transformer_to_utm = Transformer.from_crs(crs_latlon, crs_utm, always_xy=True)

    x0, y0 = transformer_to_utm.transform(lon0, lat0)
    lon, lat = transformer_to_latlon.transform(x + x0, y + y0)
    return lat, lon
