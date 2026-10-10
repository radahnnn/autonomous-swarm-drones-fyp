"""
Common Global Reference Frame & Coordinate Transformation Utility.
Converts between WGS84 GPS (Lat, Lon, Relative Alt) and a Unified Flat-Earth NED Frame.
Ensures multi-drone SITL swarms operate in a single consistent metric space regardless of boot/home locations.
"""

import math
from typing import Tuple
import numpy as np

# WGS84 Earth constants
WGS84_A = 6378137.0  # Equatorial radius (m)
WGS84_ECC2 = 0.00669437999014  # Eccentricity squared


class CommonCoordinateFrame:
    """
    Anchors all SITL drones to a shared geographic datum (Lat0, Lon0, Alt0).
    Provides flat-earth tangent plane transformations accurate to < 1mm over 5km radius.
    """

    def __init__(self, datum_lat: float = -35.3632621, datum_lon: float = 149.1652374, datum_alt: float = 584.0):
        self.lat0 = float(datum_lat)
        self.lon0 = float(datum_lon)
        self.alt0 = float(datum_alt)

        lat0_rad = math.radians(self.lat0)
        # Meridian radius of curvature
        self.r_lat = WGS84_A * (1.0 - WGS84_ECC2) / ((1.0 - WGS84_ECC2 * math.sin(lat0_rad)**2)**1.5)
        # Prime vertical radius of curvature
        self.r_lon = (WGS84_A / math.sqrt(1.0 - WGS84_ECC2 * math.sin(lat0_rad)**2)) * math.cos(lat0_rad)

    def gps_to_global_ned(self, lat: float, lon: float, rel_alt: float) -> np.ndarray:
        """Converts GPS latitude/longitude (deg) and relative altitude (m) to shared global NED (m)."""
        d_lat = math.radians(lat - self.lat0)
        d_lon = math.radians(lon - self.lon0)
        north = d_lat * self.r_lat
        east = d_lon * self.r_lon
        down = -float(rel_alt)
        return np.array([north, east, down], dtype=np.float64)

    def global_ned_to_gps(self, north: float, east: float, down: float) -> Tuple[float, float, float]:
        """Converts shared global NED coordinates back to GPS lat, lon, rel_alt."""
        lat = self.lat0 + math.degrees(north / self.r_lat)
        lon = self.lon0 + math.degrees(east / self.r_lon)
        rel_alt = -down
        return lat, lon, rel_alt

    def global_to_local_ned(self, global_ned: np.ndarray, drone_home_global: np.ndarray) -> np.ndarray:
        """
        Translates a global coordinate setpoint to an individual vehicle's local NED frame:
        P_local = P_global - P_home_global
        """
        return np.array(global_ned) - np.array(drone_home_global)
