from utilites import *
import logging
import dronekit


logger = logging.getLogger(__name__)


height = 5
target_geo_location = 50.443326, 30.4480785
target_location_error = 2
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
    drone.channels.overrides['3'] = 2000
logger.info('vehicle took off')

target_location = dronekit.LocationGlobalRelative(target_geo_location[0], target_geo_location[1], height)
rotate_to_yaw(drone, get_rotation(drone, target_location), 5)
logger.info('vehicle is rotated')

move_to(drone, target_location, target_location_error, 3)

drone.channels.overrides['2'] = 1500
logger.info('vehicle has been arrived')

rotate_to_yaw(drone, finish_yaw, 1)

drone.channels.overrides['4'] = None
logger.info('finish')

drone.close()

