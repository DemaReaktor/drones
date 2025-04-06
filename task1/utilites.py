from math import radians, cos, sin, sqrt, atan2
from dronekit import VehicleMode, Vehicle
from pymavlink import mavutil
import logging


logger = logging.getLogger(__name__)


def get_distance_meters(lat1, lon1, lat2, lon2):
    radius = 6378137.0
    dLat = radians(lat2 - lat1)
    dLon = radians(lon2 - lon1)
    a = sin(dLat / 2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dLon / 2)**2
    return 2 * atan2(sqrt(a), sqrt(1 - a)) * radius


def get_ned_vector(target_location, home_location):
    dNorth = get_distance_meters(home_location.lat, home_location.lon, target_location.lat, home_location.lon)
    dEast = get_distance_meters(home_location.lat, home_location.lon, home_location.lat, target_location.lon)
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
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        pass

    logger.info('vehicle is armed')


def take_off(vehicle: Vehicle, altitude: float):
    vehicle.simple_takeoff(altitude)

    while vehicle.location.global_relative_frame.alt < altitude:
        pass
    logger.info('vehicle is flying')


def send_ned_velocity(vehicle: Vehicle, velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    vehicle.send_mavlink(msg)


def send_yaw(vehicle: Vehicle, yaw):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        yaw,  # param 1, yaw in degrees
        15,  # param 2, yaw speed deg/s
        1,  # param 3, direction -1 ccw, 1 cw
        0,  # param 4, relative offset 1, absolute angle 0
        0, 0, 0)  # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)
