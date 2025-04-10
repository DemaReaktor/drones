import time
from math import radians, cos, sin, sqrt, atan2, degrees
from dronekit import VehicleMode, Vehicle, LocationGlobalRelative
import logging
from typing import Tuple

logger = logging.getLogger(__name__)


def keep_height(vehicle: Vehicle):
    vehicle.channels.overrides['3'] = 1500


def get_distance_meters(first_geo_location: LocationGlobalRelative, second_geo_location: LocationGlobalRelative) -> (
        float):
    radius = 6378137.0
    lat_delta = radians(second_geo_location.lat - first_geo_location.lat)
    lon_delta = radians(second_geo_location.lon - first_geo_location.lon)
    a = (sin(lat_delta / 2)**2 + cos(radians(first_geo_location.lat)) * cos(radians(second_geo_location.lon))
         * sin(lon_delta / 2)**2)
    return 2 * atan2(sqrt(a), sqrt(1 - a)) * radius


def get_vector(home_location: LocationGlobalRelative, target_location: LocationGlobalRelative) -> (
        Tuple[float, float, float]):
    dNorth = get_distance_meters(home_location, LocationGlobalRelative(target_location.lat, home_location.lon))
    dEast = get_distance_meters(home_location, LocationGlobalRelative(home_location.lat, target_location.lon))
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


def move_to(vehicle: Vehicle, target_location: LocationGlobalRelative, target_location_error: float = 5,
    rotation_error: float = 5):
    target_distance = get_distance_meters(vehicle.location.global_frame, target_location)
    rotation = get_rotation_from_yaw(vehicle, get_rotation(vehicle, target_location))
    while target_distance > target_location_error:
        keep_height(vehicle)
        vehicle.channels.overrides['4'] = (1500 + int(rotation * 0.7 + (20 if rotation >= 0 else -20))) if (
            not -rotation_error < rotation < rotation_error) else None
        vehicle.channels.overrides['2'] = max(1000, 1473 - int(target_distance * 3.5))
        time.sleep(0.1)
        rotation = get_rotation_from_yaw(vehicle, get_rotation(vehicle, target_location))
        target_distance = get_distance_meters(vehicle.location.global_frame, target_location)


def rotate_to_yaw(vehicle: Vehicle, yaw: float, rotation_error: float = 1):
    rotation = get_rotation_from_yaw(vehicle, yaw)
    while not -rotation_error < rotation < rotation_error:
        keep_height(vehicle)
        vehicle.channels.overrides['4'] = (1500 + int(rotation * 0.4 + (25 if rotation >= 0 else -25))) if (
            not -rotation_error < rotation < rotation_error) else 1500
        time.sleep(0.1)
        rotation = get_rotation_from_yaw(vehicle, yaw)


def get_rotation_from_yaw(vehicle: Vehicle, yaw: float):
    return (yaw - degrees(vehicle.attitude.yaw) + 540) % 360 - 180


def get_rotation(vehicle: Vehicle, position: LocationGlobalRelative) -> float:
    vector = get_vector(vehicle.location.global_frame, position)
    return (450 - degrees(atan2(vector[0], vector[1]))) % 360
