from math import sqrt, atan2, degrees
from geopy.distance import distance
from geopy.point import Point


drone_yaw = 335
target_geo_location = 50.603694, 30.650625
target_pixels_location = 558, 328
home_pixels_location = 320, 256
pixels_to_meters_coefficient = 0.38

pixels_vector = home_pixels_location[0] - target_pixels_location[0], home_pixels_location[1] - target_pixels_location[1]
vector_meters = pixels_vector[0] * pixels_to_meters_coefficient, pixels_vector[1] * pixels_to_meters_coefficient
delta = sqrt(vector_meters[0]**2 + vector_meters[1]**2)
start = Point(target_geo_location[0], target_geo_location[1])
bearing = degrees(atan2(vector_meters[0], vector_meters[1]))
point = distance(meters=delta).destination(point=start, bearing=180+drone_yaw-bearing)
result = point.latitude, point.longitude

print(result)
