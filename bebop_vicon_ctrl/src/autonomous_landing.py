#!/usr/bin/env python
import roslib
roslib.load_manifest('teleop_twist_keyboard')
import rospy
import tf

from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Empty, String, Float64, Bool

vicon_x = 0
vicon_y = 0
vicon_z = 0
vicon_yaw = 0

goal_x = 0
goal_y = 0
control_z = 0
enable_goal = False
enable_auto_land = False


def update_enable_goal(data):
    global enable_goal
    enable_goal = data.data


def update_enable_auto_land(data):
    global enable_auto_land
    enable_auto_land = data.data


def vicon_callback(data):
    global vicon_x, vicon_y, vicon_z, vicon_yaw

    translations = data.transform.translation
    quat = data.transform.rotation
    # print(translations)
    vicon_x = translations.x
    vicon_y = translations.y
    vicon_z = translations.z


def update_goal_x(data):
    global goal_x
    goal_x = data.data


def update_goal_y(data):
    global goal_y
    goal_y = data.data


goal_location = None

really_close_value = 0.04
min_altitude = 1.0


def reallyClose():
    global vicon_x, vicon_y, vicon_z, goal_x, goal_y
    # print(vicon_x, vicon_y, goal_x, goal_y)
    error = abs(vicon_x - goal_x) + abs(vicon_y - goal_y)
    # print(error)
    if abs(vicon_x - goal_x) < really_close_value and abs(vicon_y - goal_y) < really_close_value and vicon_z < min_altitude:
        return True
    else:
        return False


if __name__ == "__main__":

    rospy.init_node('autonomous_landing')

    bebop_ns = rospy.get_param("/bebop_ns")

    pub_land = rospy.Publisher(bebop_ns + '/land', Empty, queue_size=10)

    pub_enable_goal = rospy.Publisher(
        bebop_ns + '/setpoint/enable', Bool, queue_size=10)
    pub_sound = rospy.Publisher('/sound_effects', String, queue_size=10)

    sub = rospy.Subscriber("/vicon/bebop1/bebop1",
                           TransformStamped, vicon_callback)

    sub_enable_goal = rospy.Subscriber(bebop_ns + '/setpoint/enable',
                                       Bool, update_enable_goal)

    sub_enable_auto_land = rospy.Subscriber(bebop_ns + '/setpoint/enable_auto_land',
                                            Bool, update_enable_auto_land)

    sub_x_goal = rospy.Subscriber(
        bebop_ns + '/setpoint/x', Float64, update_goal_x)
    sub_y_goal = rospy.Subscriber(
        bebop_ns + '/setpoint/y', Float64, update_goal_y)

    while not rospy.is_shutdown():

        reallyClose()

        if enable_goal and enable_auto_land:

            if reallyClose():
                pub_enable_goal.publish(Bool(False))
                pub_land.publish(Empty())
                pub_sound.publish(String('landing_beep'))
