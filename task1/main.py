import dronekit
import math
import time
from dronekit import connect
from utilites import *
import logging


logger = logging.getLogger(__name__)


height = 6
target_geo_location = 50.443326, 30.4480785
target_location_error = 2
max_speed = 10
min_speed = 2
connection_string = "tcp:127.0.0.1:5762"
finish_yaw = 350


formatter = logging.Formatter(
    "[{asctime}:{levelname}:{name}] {message}", style="{", datefmt="%d %H:%M:%S"
)
logging.basicConfig(level=logging.INFO,
    format="[{asctime}:{levelname}:{name}] {message}",
    style="{",
    datefmt="%d %H:%M:%S")


drone = connect(connection_string, wait_ready=True)
prepare(drone)
take_off(drone, height)

target_location = dronekit.LocationGlobalRelative(target_geo_location[0], target_geo_location[1], height)
while True:
    north, east, down = get_ned_vector(target_location, drone.location.global_frame)
    length = math.sqrt(north*north + east*east)
    if length < target_location_error:
        break
    if length > max_speed * 1.5:
        send_ned_velocity(drone, north / length * max_speed, east / length * max_speed, 0)
    else:
        send_ned_velocity(drone, north / length * min_speed, east / length * min_speed, 0)
    time.sleep(0.1)

logger.info('vehicle has been arrived')

send_yaw(drone, finish_yaw)
logger.info('vehicle start rotating')
drone.close()

