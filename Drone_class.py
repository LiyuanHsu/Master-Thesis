



import rospy
import numpy as np
import matplotlib.pyplot as plt
import tensorflow as tf
import time
import gym
from std_msgs.msg import String, Float64, Int16
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped


# For Agenet and Gym
import sys
sys.path.insert(0, '/home/lhsu/PycharmProjects/hierarchical_TL-python_2/HL/Curriculum_training/model_one_step')
sys.path.insert(0, '/home/lhsu/PycharmProjects/hierarchical_TL-python_2/HL/Curriculum_training')
sys.path.insert(0, '/home/lhsu/PycharmProjects/hierarchical_TL-python_2/HL')
sys.path.insert(0, '/home/lhsu/PycharmProjects/hierarchical_TL-python_2')

from HL.TrainerPolicySingleOut import TrainerPolicySingleOut
from HL.Architectures.LidarGoalFullyConnected import LidarGoalFullyConnected
from HL.OneHiddenFull import OneHiddenFull
from HL.Worker import Worker
import multiprocessing




class Quadrotor(object):

	def __init__(self, policies, sess):
   
		self.sensor_subscriber = rospy.Subscriber("scan", LaserScan, self.lidar_data_cb)
		self.sensor_sbuscriber = rospy.Subscriber("vicon/cylinder1/cylinder1", TransformStamped, self.cylinder1_cb)
		self.sensor_sbuscriber = rospy.Subscriber("vicon/cylinder2/cylinder2", TransformStamped, self.cylinder2_cb)
		self.sensor_sbuscriber = rospy.Subscriber("vicon/cylinder3/cylinder3", TransformStamped, self.cylinder3_cb)


		self.sensor_sbuscriber = rospy.Subscriber("vicon/Obstacle_rectangle/Obstacle_rectangle", TransformStamped, self.rectangle_cb)
		self.sensor_subscriber = rospy.Subscriber("vicon/bebop103/bebop103", TransformStamped, self.drone_cb)

                self.pub_action = rospy.Publisher("bebop/action", Int16, queue_size=1)

		self.flight_done = False

		self.policies = policies
		self.sess = sess
	
		print("Into constructor of Quadrotor")
		self.i = 0

		# Plot lidar_readings
		self.lidar_receive = False



	def lidar_data_cb(self, data):
		self.lidar_readings = np.asarray(data.ranges)
		self.lidar_readings = self.lidar_readings[(self.lidar_readings < 40)]
		self.lidar_readings = np.around(self.lidar_readings, decimals=2)
		self.lidar_readings[self.lidar_readings == 0.01] = 2
		self.lidar_readings = self.lidar_readings[0:140]
		self.lidar_readings = self.lidar_readings[0::4]
		self.lidar_readings[self.lidar_readings > 2] = 2
		self.lidar_readings[self.lidar_readings == 0.05] = 2
		self.lidar_receive = True
		# print (self.lidar_readings)
		print (type(data.ranges))
		print (len(self.lidar_readings))
		print (type(self.lidar_readings))



		# Plot lidar_readings
		# plt.clf()
		# plt.ion()
		# # plt.axis([-5, 40, 0, 5])
		# plt.scatter(self.x, lidar_readings)
		# plt.show()
		# plt.pause(0.0001)
		# # plt.pause(0.0000001)


		# Calculate action
		self.action = self.policies[0].inference(self.sess, self.lidar_readings)
		print("action:", self.action)

                # Publish action
                self.pub_action.publish(Int16(self.action))


		print("I got the message", self.i)
		self.message = data
		self.i=self.i+1
		# if self.i > 5:
		# 	self.flight_done = True

	def cylinder1_cb(self, data):
		self.cylinder1_x = data.transform.translation.x
		self.cylinder1_y = data.transform.translation.y

	def cylinder2_cb(self, data):
		self.cylinder2_x = data.transform.translation.x
		self.cylinder2_y = data.transform.translation.y

	def cylinder3_cb(self, data):
		self.cylinder3_x = data.transform.translation.x
		self.cylinder3_y = data.transform.translation.y

	def rectangle_cb(self, data):
		self.rectangle_x = data.transform.translation.x
		self.rectangle_y = data.transform.translation.y

	def drone_cb(self, data):
		self.drone_x = data.transform.translation.x
		self.drone_y = data.transform.translation.y



