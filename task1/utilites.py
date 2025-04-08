from math import radians, cos, sin, sqrt, atan2, degrees
from dronekit import VehicleMode, Vehicle, LocationGlobalRelative
import logging

logger = logging.getLogger(__name__)


def get_distance_meters(first_geo_location, second_geo_location):
    radius = 6378137.0
    lat_delta = radians(second_geo_location[0] - first_geo_location[0])
    lon_delta = radians(second_geo_location[1] - first_geo_location[1])
    a = (sin(lat_delta / 2)**2 + cos(radians(first_geo_location[0])) * cos(radians(second_geo_location[1]))
         * sin(lon_delta / 2)**2)
    return 2 * atan2(sqrt(a), sqrt(1 - a)) * radius


def get_ned_vector(target_location, home_location):
    dNorth = get_distance_meters((home_location.lat, home_location.lon), (target_location.lat, home_location.lon))
    dEast = get_distance_meters((home_location.lat, home_location.lon), (home_location.lat, target_location.lon))
    if target_location.lat < home_location.lat:
        dNorth *= -1
    if target_location.lon < home_location.lon:
        dEast *= -1
    dDown = home_location.alt - target_location.alt
    return dNorth, dEast, dDown


def prepare(vehicle: Vehicle):
    while not vehicle.is_armable:
        pass

    logger.info('vehicle is armable')
    vehicle.mode = VehicleMode("ALT_HOLD")
    vehicle.arm()

    while not vehicle.armed:
        pass

    logger.info('vehicle is armed')


def rotate(vehicle: Vehicle, right: bool = True):
    vehicle.channels.overrides['4'] = 1540 if right else 1460


def get_rotation(vehicle: Vehicle, position: LocationGlobalRelative) -> float:
    vector = get_ned_vector(position, vehicle.location.global_frame)
    return degrees(atan2(vector[1], vector[0]))