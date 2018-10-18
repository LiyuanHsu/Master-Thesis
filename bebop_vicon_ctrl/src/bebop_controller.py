#!/usr/bin/env python
import roslib
roslib.load_manifest('teleop_twist_keyboard')
import rospy
import tf

from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty, String, Float64, Bool

import sys
import select
import termios
import tty
import time
import thread
import math

vicon_x = 0
vicon_y = 0
vicon_z = 0
vicon_yaw = 0

control_x = 0
control_y = 0
control_z = 0
control_yaw = 0
enable_goal = False
land_attempt = False


def update_x(data):
    global control_x
    control_x = data.data
    # print(control_x)


def update_y(data):
    global control_y
    control_y = data.data
    # print(control_y)


def update_z(data):
    global control_z
    control_z = data.data


def update_yaw(data):
    global control_yaw
    control_yaw = data.data


def update_enable_goal(data):
    global enable_goal
    enable_goal = data.data


def vicon_callback(data):
    global vicon_x, vicon_y, vicon_z, vicon_yaw

    translations = data.transform.translation
    quat = data.transform.rotation
    vicon_x = translations.x
    vicon_y = translations.y
    vicon_z = translations.z

    # Convert quaternions to Euler angles.
    (r, p, y) = tf.transformations.euler_from_quaternion(
        [quat.x, quat.y, quat.z, quat.w])

    vicon_yaw = y

    # pub_calculated.publish(
    # String('x:' + str(vicon_x) + ' y:' + str(vicon_y) + ' yaw:' +
    # str(vicon_yaw)))
    pub_x.publish(Float64(vicon_x))
    pub_y.publish(Float64(vicon_y))
    pub_z.publish(Float64(vicon_z))
    pub_yaw.publish(Float64(vicon_yaw))


goal_location = None

breakNow = False

# def contoller():

# def reallyClose():


if __name__ == "__main__":

    rospy.init_node('bebop_controller')
    rate = rospy.Rate(100)

    bebop_ns = rospy.get_param("/bebop_ns")

    pub_land = rospy.Publisher(bebop_ns + '/land', Empty, queue_size=10)
    pub = rospy.Publisher(bebop_ns + '/cmd_vel', Twist, queue_size=1)
    pub_x = rospy.Publisher(bebop_ns + '/state/x', Float64, queue_size=1)
    pub_y = rospy.Publisher(bebop_ns + '/state/y', Float64, queue_size=1)
    pub_z = rospy.Publisher(bebop_ns + '/state/z', Float64, queue_size=1)
    pub_yaw = rospy.Publisher(bebop_ns + '/state/yaw', Float64, queue_size=1)

    sub = rospy.Subscriber("vicon/bebop103/bebop103",
                           TransformStamped, vicon_callback)

    sub_enable_goal = rospy.Subscriber(bebop_ns + '/setpoint/enable',
                                       Bool, update_enable_goal)
    sub_x_effort = rospy.Subscriber(
        bebop_ns + '_x/control_effort', Float64, update_x)

    sub_y_effort = rospy.Subscriber(
        bebop_ns + '_y/control_effort', Float64, update_y)
    sub_z_effort = rospy.Subscriber(
        bebop_ns + '_z/control_effort', Float64, update_z)

    sub_yaw_effort = rospy.Subscriber(
        bebop_ns + '_yaw/control_effort', Float64, update_yaw)

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    turn = 1.0
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    prop_k = 0.1
    limit_vel = 0.1

    global control_x, control_y, control_z, control_yaw

    try:

        while not rospy.is_shutdown():

            # I had a delay of 1/10 second in the control loop?!@? - WTF mate
            # time.sleep(0.1)
            
            if enable_goal:

                dx_vicon = control_x
                dy_vicon = control_y

                dx_quadcopter = dx_vicon * \
                    math.cos(vicon_yaw) + dy_vicon * math.sin(vicon_yaw)
                dy_quadcopter = - dx_vicon * \
                    math.sin(vicon_yaw) + dy_vicon * math.cos(vicon_yaw)

                x = dx_quadcopter
                y = dy_quadcopter
                z = control_z
                th = control_yaw

                # print('position driven to goal ', goal_location)

            else:
                x = 0
                y = 0
                z = 0
                th = 0

            twist = Twist()
            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th * turn
            pub.publish(twist)

            rate.sleep()

    except rospy.ROSException:
        print('exception')
        # print e

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)
