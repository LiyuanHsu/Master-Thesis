



import rospy
import numpy as np
import tensorflow as tf
import time
import gym
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan


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

	def __init__(self):
		# Agent and Gym
		self.max_episode_length = 1000
		self.gamma = .99 # discount rate for advantage estimation and reward discounting
		self.load_model = True

		self.record_gif = False
		self.model_path = '/home/lhsu/PycharmProjects/hierarchical_TL-python_2/HL/Curriculum_training/model_one_step'

		self.POLICY_PARAM = {
   		 'n_bootstraping': 60,  # THIS IS TEST!!! DO NOT HAVE INFLUENCE HERE!!!
		}

		self.game_name = 'CylinderDiffSizesEnv-v0'

		self.ENV_PARMAS = {
 	   	'seed': 0,
 	   	'set_seed': False,
 	   	'obstacle_radius': 0.05,
  	   	'obstacle_num': 1,
  	   	'obstacle_type': ['cylinder'],
  	   	'set_obst_pose': False,
		}

   
		self.sensor_subscriber = rospy.Subscriber("scan", LaserScan, self.subscriber_cb)
		self.flight_done = False
	
		print("Into constructor of Quadrotor")
		self.i = 0


	def subscriber_cb(self, data):
		lidar_readings = np.asarray(data.ranges)
		lidar_readings = lidar_readings[(lidar_readings < 40)]
		lidar_readings = np.around(lidar_readings, decimals=2)
		lidar_readings[lidar_readings == 0.01] = 2
		lidar_readings = lidar_readings[0:140]
		lidar_readings = lidar_readings[0::4]
		print (lidar_readings)
		print (type(data.ranges))
		print (len(lidar_readings))
		print (type(lidar_readings))

		self.replay(lidar_readings)

		print("I got the message", self.i)
		self.message = data
		self.i=self.i+1
		if self.i > 5:
			self.flight_done = True


	def device(self):
		print("Into device...")
		with tf.device("/cpu:0"):
			global_episodes = tf.Variable(0, dtype=tf.int32, name='global_episodes', trainable=False)
			optimizer = tf.train.AdamOptimizer(learning_rate=1e-4) #learning_rate=1e-4
			self.game = gym.make(self.game_name)
			self.game.seed(self.ENV_PARMAS)
			a_size = self.game.action_space.n
			s_size = len(self.game.observation_space.high)



			master_network = OneHiddenFull(s_size, a_size, 'global')
			# master_network = LidarGoalFullyConnected(s_size, 0, a_size, 'global')
			num_workers = multiprocessing.cpu_count()  # Set workers ot number of available CPU threads
			workers = []
			self.policies = []
			trainers = []

			self.saver = tf.train.Saver(max_to_keep=5)

			for i in range(1):
				self.game = gym.make(self.game_name)
				self.game.seed(self.ENV_PARMAS)
                a_size = self.game.action_space.n
                s_size = len(self.game.observation_space.high)
                self.policies.append(OneHiddenFull(s_size, a_size, "worker_" + str(i)))
                # policies.append(LidarGoalFullyConnected(s_size, 0, a_size, "worker_" + str(i)))
                trainers.append(TrainerPolicySingleOut(self.policies[-1], optimizer))
                workers.append(Worker(self.game, i, trainers[-1], self.model_path, global_episodes, self.POLICY_PARAM))



	def replay(self, lidar_readings):
		with tf.Session() as self.sess:
			coord = tf.train.Coordinator()
			if self.load_model == True:
				print('Loading Model...')
				ckpt = tf.train.get_checkpoint_state(self.model_path)
				self.saver.restore(self.sess, ckpt.model_checkpoint_path)
			else:
				self.sess.run(tf.global_variables_initializer())

			action = self.policies[0].inference(self.sess, lidar_readings)
			print("action: ", action)

			# test_game = gym.make(self.game_name)
			# test_game.seed(self.ENV_PARMAS)
			# reward_sum = 0.0
            #
			# if not self.record_gif:
			# 	for i_episode in range(3):
			# 		observation = test_game.reset()
			# 		for t in range(self.max_episode_length):
			# 			test_game.render()
			# 			if (t == 0):
			# 				time.sleep(1.5)
			# 			else:
			# 				time.sleep(0.01)
            #
			# 			action = self.policies[0].inference(sess, observation)
			# 			# action, _ = policies[0].explore(sess, observation)
			# 			print("action:", action)
			# 			observation, reward, done, info = test_game.step(action)
			# 			# observation, reward, done, info = test_game.step(0)
			# 			# print(observation)
			# 			reward_sum += reward
			# 			# print(observation)
			# 			# print("reward:", reward)
			# 			if done:
			# 				print("Episode finished after {} timesteps".format(t + 1))
			# 				print(reward_sum)
			# 				break
		









