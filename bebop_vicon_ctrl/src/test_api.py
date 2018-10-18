#!/usr/bin/env python

from bebop_api_client import Bebop
import time

# make bebop client object
bebop1 = Bebop('bebop1')

# takeoff command - no input
bebop1.takeoff()

time.sleep(5)

path = [(-2, 1.4), (-1.9, 0.3), (-1.39, 0.6),
        (-1.25, 1.43), (-0.72, 1.35), (-.58, 0.65)]

for point in path:
fly_to command - (x, y, z) in vicon world frame
bebop1.fly_to(point[0], point[1], 0.9)
time.sleep(4)

time.sleep(3)


# Two landing options:

# land command - no input
bebop1.land()

# land on turtlebot command - no input
bebop1.turtlebot_land()
