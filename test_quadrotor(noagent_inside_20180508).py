




#import OneHiddenFull
#import Worker
#import test_lidar_ros20180507

import rospy
import time
import tensorflow as tf
import gym
from Drone_class import Quadrotor


tf.reset_default_graph()

#for t in range(10):

    #test_game.render()
    #test_game.step(0)
    #time.sleep(1)


print("GOGOYAYAYA")



rospy.init_node('test_quadrotor')
quadrotor = Quadrotor()
quadrotor.device()


rate_on = rospy.Rate(1)

while not rospy.is_shutdown():
    rate_on.sleep()
    if quadrotor.flight_done:
        break



#quadrotor.subscriber_cb()


print("Finish fling")



