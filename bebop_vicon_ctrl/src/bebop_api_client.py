#!/usr/bin/env python
import roslib
roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_srvs.srv import Empty as EmptySrv
from bebop_vicon_control.srv import FlyTo

from std_msgs.msg import Empty, String, Float64, Bool
from geometry_msgs.msg import TransformStamped

import thread
turtle_pad_x = 0
turtle_pad_y = 0


def turtle_pad_callback(data):
    global turtle_pad_x, turtle_pad_y
    turtle_pad_x = data.transform.translation.x
    turtle_pad_y = data.transform.translation.y


class Bebop(object):
    """A customer of ABC Bank with a checking account. Customers have the
    following properties:

    Attributes:
        name: A string representing the customer's name.
        balance: A float tracking the current balance of the customer's account.
    """

    def __init__(self, name):
        """Return a Customer object whose name is *name* and starting
        balance is *balance*."""
        self.name = name
        bebop_ns = name

    def takeoff(self):
        """Return the balance remaining after withdrawing *amount*
        dollars."""
        print('**Takeoff**')
        takeoff_call = rospy.ServiceProxy('bebop1/takeoff', EmptySrv)
        takeoff_call()
        return True

    def land(self):
        """Return the balance remaining after withdrawing *amount*
        dollars."""
        print('**Landing**')
        land_call = rospy.ServiceProxy('bebop1/land', EmptySrv)
        land_call()
        return True

    def turtlebot_land(self):
        """Return the balance remaining after withdrawing *amount*
        dollars."""
        print('**Landing on Turtlebot**')
        land_call = rospy.ServiceProxy('bebop1/turtlebot_land', EmptySrv)
        land_call()
        return True

    def fly_to(self, x, y, z):
        """Return the balance remaining after depositing *amount*
        dollars."""
        theta = 0
        print("fly to " + str(x) + ',' + str(y) + ',' + str(z))
        fly_to_call = rospy.ServiceProxy('bebop1/fly_to', FlyTo)
        response = fly_to_call(x, y, z, theta)
        print(response)
        return response

    def chase(self):
        """Return the balance remaining after depositing *amount*
        dollars."""
        fly_to_call = rospy.ServiceProxy('bebop1/chase', EmptySrv)
        fly_to_call()
        return True


if __name__ == "__main__":

    rospy.init_node('bebop_api')

    bebop1 = Bebop('bebop1')

    rospy.spin()
