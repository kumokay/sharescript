import math
from AirSimClient import *


def degreesToRadians(degrees):
    return math.pi * degrees / 180.0

def GeodeticToNedFast(geo, home):
    d_lat = geo.latitude - home.latitude;
    d_lon = geo.longitude - home.longitude;
    d_alt = home.altitude - geo.altitude;
    EARTH_RADIUS = 6378137.0
    x = degreesToRadians(d_lat) * EARTH_RADIUS
    y = degreesToRadians(d_lon) * EARTH_RADIUS * math.cos(degreesToRadians(geo.latitude))
    return Vector3r(x, y, d_alt)

def nedToGeodeticFast(local, home):
    d_lat = local.x_val / EARTH_RADIUS;
    d_lon = local.y_val / (EARTH_RADIUS * math.cos(degreesToRadians(home.latitude)));
    latitude = home.latitude + radiansToDegrees(d_lat);
    longitude = home.longitude + radiansToDegrees(d_lon);
    altitude = home.altitude - local.z_val;
    return GeoPoint(latitude,longitude,altitude)

def getCalibratedPos(pos, ref):
    return Vector3r(pos.x_val - ref.x_val, pos.y_val - ref.y_val, pos.z_val - ref.z_val)

def getCalibratedPosFromGps(gps, home, ref):
    pos = GeodeticToNedFast(gps, home)
    return Vector3r(pos.x_val - ref.x_val, pos.y_val - ref.y_val, pos.z_val - ref.z_val)

def getDistance(pos, ref):
    return Vector3r(pos.x_val - ref.x_val, pos.y_val - ref.y_val, pos.z_val - ref.z_val).length()
