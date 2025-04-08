import math
from utilites import *
import logging
import time
import dronekit


logger = logging.getLogger(__name__)


height = 100
target_geo_location = 50.443326, 30.4480785
target_location_error = 5
speed = 10
connection_string = "tcp:127.0.0.1:5762"
finish_yaw = 350


formatter = logging.Formatter(
    "[{asctime}:{levelname}:{name}] {message}", style="{", datefmt="%d %H:%M:%S"
)
logging.basicConfig(level=logging.INFO,
    format="[{asctime}:{levelname}:{name}] {message}",
    style="{",
    datefmt="%d %H:%M:%S")


drone = dronekit.connect(connection_string, wait_ready=True)
prepare(drone)
while drone.location.global_relative_frame.alt < height:
    drone.channels.overrides['3'] = 1800
logger.info('vehicle took off')
while True:
    drone.channels.overrides['3'] = 1500
    target_location = dronekit.LocationGlobalRelative(target_geo_location[0], target_geo_location[1], height)
    rotation = get_rotation(drone, target_location)
    if math.fabs(rotation - degrees(drone.attitude.yaw)) < 5:
        break
    rotate(drone, True)
    time.sleep(0.2)
logger.info('vehicle is rotated')

drone.channels.overrides['4'] = None
target_location = dronekit.LocationGlobalRelative(target_geo_location[0], target_geo_location[1], height)

while True:
    drone.channels.overrides['3'] = 1500
    north, east, down = get_ned_vector(target_location, drone.location.global_frame)
    length = math.sqrt(north**2 + east**2)
    if length < target_location_error:
        break
    drone.channels.overrides['2'] = max(1000, min(2000, 1500 - int(speed * 100)))
    rotation = get_rotation(drone, target_location)
    if math.fabs(rotation - degrees(drone.attitude.yaw)) > 5:
        rotate(drone, True)
    else:
        drone.channels.overrides['4'] = None
    time.sleep(0.2)
logger.info('vehicle has been arrived')
drone.channels.overrides['2'] = None

while True:
    if math.fabs((finish_yaw - degrees(drone.attitude.yaw) + 180) % 360 - 180) < 5:
        break
    drone.channels.overrides['4'] = 1525 if 0 < (finish_yaw - degrees(drone.attitude.yaw) + 360) % 360 < 180 else 1475

drone.channels.overrides['4'] = None
logger.info('finish')
drone.close()

