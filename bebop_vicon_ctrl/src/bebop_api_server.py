#!/usr/bin/env python
import roslib
roslib.load_manifest('teleop_twist_keyboard')
import rospy

from std_srvs.srv import Empty as EmptySrv
from bebop_vicon_control.srv import FlyTo
from std_msgs.msg import Empty, String, Float64, Bool
from geometry_msgs.msg import TransformStamped


import thread
import time
turtle_pad_x = 0
turtle_pad_y = 0


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

        # Also set up some ROS services
        rospy.Service('bebop1/takeoff', EmptySrv,
                      self.takeoff_service_callback)
        rospy.Service('bebop1/land', EmptySrv,
                      self.land_service_callback)
        rospy.Service('bebop1/turtlebot_land', EmptySrv,
                      self.turtlebot_land_service_callback)
        rospy.Service('bebop1/fly_to', FlyTo,
                      self.fly_to_service_callback)
        rospy.Service('bebop1/chase', EmptySrv,
                      self.chase_service_callback)

        self.sub_pad = rospy.Subscriber(
            "vicon/turtle_pad/turtle_pad", TransformStamped, self.turtle_pad_callback)

        self.pub_x_goal = rospy.Publisher(
            bebop_ns + '/setpoint/x', Float64, queue_size=10)
        self.pub_y_goal = rospy.Publisher(
            bebop_ns + '/setpoint/y', Float64, queue_size=10)
        self.pub_z_goal = rospy.Publisher(
            bebop_ns + '/setpoint/z', Float64, queue_size=10)
        self.pub_yaw_goal = rospy.Publisher(
            bebop_ns + '/setpoint/yaw', Float64, queue_size=10)
        self.pub_enable_goal = rospy.Publisher(
            bebop_ns + '/setpoint/enable', Bool, queue_size=10)
        self.pub_takeoff = rospy.Publisher(
            '/bebop1/takeoff', Empty, queue_size=10)
        self.pub_land = rospy.Publisher(
            bebop_ns + '/land', Empty, queue_size=10)
        self.pub_emergency = rospy.Publisher(
            bebop_ns + '/reset', Empty, queue_size=10)

        self.pub_enable_auto_land = rospy.Publisher(bebop_ns + '/setpoint/enable_auto_land',
                                                    Bool, queue_size=10)

        self.turtle_pad_x = 0
        self.turtle_pad_y = 0
        self.chaseMode = False

    def turtle_pad_callback(self, data):
        self.turtle_pad_x = data.transform.translation.x
        self.turtle_pad_y = data.transform.translation.y

    def publishGoals(self, goal_location):
        print('Set goal to: ' + str(goal_location))
        self.pub_x_goal.publish(Float64(goal_location[0]))
        self.pub_y_goal.publish(Float64(goal_location[1]))
        self.pub_z_goal.publish(Float64(goal_location[2]))
        self.pub_yaw_goal.publish(Float64(goal_location[3]))
        self.pub_enable_goal.publish(Bool(True))

    def takeoff_service_callback(self, req):
        """Return the balance remaining after withdrawing *amount*
        dollars."""
        print('**Takeoff**')
        # goal_location = None
        self.pub_takeoff.publish(Empty())
        self.pub_enable_goal.publish(Bool(False))
        return []

    def land_service_callback(self, req):
        """Return the balance remaining after withdrawing *amount*
        dollars."""
        print('**Landing**')
        for i in range(3):
            self.pub_land.publish(Empty())
            self.pub_enable_goal.publish(Bool(False))
            time.sleep(0.2)
        return []

    def fly_to_service_callback(self, req):
        print(req)
        self.publishGoals([req.x, req.y, req.z, req.yaw])
        return "flew_to x,y,z"

    def chase_service_callback(self, req):
        self.chaseMode = True
        while self.chaseMode:
            self.publishGoals([self.turtle_pad_x, self.turtle_pad_y, 0.8, 0])
            time.sleep(0.5)
        return []

    def turtlebot_land_service_callback(self, req):
        """Return the balance remaining after withdrawing *amount*
        dollars."""
        print('**Landing on Turtlebot**')
        for i in range(5):
            self.publishGoals([self.turtle_pad_x, self.turtle_pad_y, 0.8, 0])
            self.pub_enable_goal.publish(Bool(True))
            self.pub_enable_auto_land.publish(Bool(True))
            time.sleep(0.5)
        return []

    def deposit(self, amount):
        """Return the balance remaining after depositing *amount*
        dollars."""
        self.balance += amount
        return self.balance


goal_location = None


def waitForKeyPress():

    global goal_location, turtle_pad_x, turtle_pad_y

    empty = Empty()
    off = Bool(False)

    # default enable goal False
    pub_enable_goal.publish(off)

    while True:
        key = getKey()
        pub_enable_auto_land.publish(Bool(False))

        if key == '\x03':  # control-c!
            print('### Reset ###')
            goal_location = None
            pub_emergency.publish(empty)
            pub_enable_goal.publish(off)
            break

        elif key == '0':
            print('### Reset ###')
            goal_location = None
            pub_emergency.publish(empty)
            pub_enable_goal.publish(off)

        elif key == '1':
            print('**Takeoff**')
            goal_location = None
            pub_takeoff.publish(empty)
            pub_enable_goal.publish(off)

        elif key == '2':
            print('**Landing**')
            goal_location = None
            pub_land.publish(empty)
            pub_enable_goal.publish(off)

        elif key == '3':
            goal_location = [-1.0, 1.0, 0.75, 1]
            publishGoals(goal_location)

        elif key == '4':
            goal_location = [-1.0, 1.0, 1.3, 1]
            publishGoals(goal_location)

        elif key == '5':
            goal_location = [turtle_pad_x, turtle_pad_y, 0.8, 0]
            pub_enable_auto_land.publish(Bool(True))
            publishGoals(goal_location)

        elif key == '6':
            goal_location = [-0.25, -0.25, 1.3, 1]
            publishGoals(goal_location)

        elif key == '7':
            goal_location = [-0.25, 0.25, 1.3, 1]
            publishGoals(goal_location)


if __name__ == "__main__":

    rospy.init_node('bebop_api')

    bebop1 = Bebop('bebop1')

    rospy.spin()
