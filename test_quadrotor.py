




#import OneHiddenFull
#import Worker
#import test_lidar_ros20180507

import rospy
import os
import time
import numpy as np
import tensorflow as tf
import gym
import csv
import matplotlib.pyplot as plt
from Drone_class import Quadrotor




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

#Matlab plot
num_samples_laser = 35
state = np.array([-0.5, -0.25])
drone_width = 0.2
drone_height = 0.2
rectangle_width = 0.4
rectangle_height = 0.6

recording_path = './recording'

if not os.path.exists(recording_path):
    os.makedirs(recording_path)


max_episode_length = 1000
gamma = .99  # discount rate for advantage estimation and reward discounting
load_model = True
record_gif = False
model_path = '/home/lhsu/PycharmProjects/hierarchical_TL-python_2/HL/Curriculum_training/model_one_step'

POLICY_PARAM = {
    'n_bootstraping': 60,  # THIS IS TEST!!! DO NOT HAVE INFLUENCE HERE!!!
}


game_name = 'CylinderDiffSizesEnv-v0'



ENV_PARMAS = {
    'seed': 0,
    'set_seed': False,
    'obstacle_radius': 0.05,
    'obstacle_num': 1,
    'obstacle_type': ['cylinder'],
    'set_obst_pose': False,
}

tf.reset_default_graph()

#for t in range(10):

    #test_game.render()
    #test_game.step(0)
    #time.sleep(1)


print("GOGOYAYAYA")

with tf.device("/cpu:0"):
    global_episodes = tf.Variable(0, dtype=tf.int32, name='global_episodes', trainable=False)
    optimizer = tf.train.AdamOptimizer(learning_rate=1e-4) #learning_rate=1e-4
    game = gym.make(game_name)
    game.seed(ENV_PARMAS)
    a_size = game.action_space.n
    s_size = len(game.observation_space.high)
    
    print a_size
    print s_size
	
    master_network = OneHiddenFull(s_size, a_size, 'global')
    # master_network = LidarGoalFullyConnected(s_size, 0, a_size, 'global')
    num_workers = multiprocessing.cpu_count()  # Set workers ot number of available CPU threads
    workers = []
    policies = []
    trainers = []

    for i in range(1):
        game = gym.make(game_name)
        game.seed(ENV_PARMAS)
        a_size = game.action_space.n
        s_size = len(game.observation_space.high)
        policies.append(OneHiddenFull(s_size, a_size, "worker_" + str(i)))
        # policies.append(LidarGoalFullyConnected(s_size, 0, a_size, "worker_" + str(i)))
        trainers.append(TrainerPolicySingleOut(policies[-1], optimizer))
        workers.append(Worker(game, i, trainers[-1], model_path, global_episodes, POLICY_PARAM))

    saver = tf.train.Saver(max_to_keep=5)
    # saver = tf.train.Saver()


with tf.Session() as sess:
    coord = tf.train.Coordinator()
    if load_model == True:
        print ('Loading Model...')
        ckpt = tf.train.get_checkpoint_state(model_path)
        saver.restore(sess, ckpt.model_checkpoint_path)
    else:
        sess.run(tf.global_variables_initializer())

    test_game = gym.make(game_name)
    test_game.seed(ENV_PARMAS)
    reward_sum = 0.0

    if not record_gif:
        for i_episode in range(0):
            observation = test_game.reset()
            for t in range(max_episode_length):
                test_game.render()
                if (t == 0):
                    time.sleep(0.5)
                else:
                    time.sleep(0.05)

                action = policies[0].inference(sess, observation)
                # action, _ = policies[0].explore(sess, observation)
                print("action:", action)
                observation, reward, done, info = test_game.step(action)
                # observation, reward, done, info = test_game.step(0)
                # print(observation)
                reward_sum += reward
                #print(observation)
                #print("reward:", reward)
                if done:
                    print("Episode finished after {} timesteps".format(t + 1))
                    print(reward_sum)
                    break



    rospy.init_node('test_quadrotor')
    quadrotor = Quadrotor(policies, sess)




    plt.ion()
    fig, ax = plt.subplots()
    x = np.arange(35)


    lidar_all_list = []
    iter = 0

    rate_on = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate_on.sleep()


        #cylinder1 = plt.Circle((quadrotor.cylinder1_x, quadrotor.cylinder1_y), 0.05, color='red')
	#cylinder2 = plt.Circle((quadrotor.cylinder2_x, quadrotor.cylinder2_y), 0.05, color='red')
	#cylinder3 = plt.Circle((quadrotor.cylinder3_x, quadrotor.cylinder3_y), 0.05, color='red')

        #rectangle = plt.Rectangle((quadrotor.rectangle_x - rectangle_width / 2, quadrotor.rectangle_y - rectangle_height / 2), \
        #                      rectangle_width, rectangle_height, fc='red', zorder=0)


        drone = plt.Rectangle((quadrotor.drone_x - drone_width / 2, quadrotor.drone_y - drone_height / 2), \
                                  drone_width, drone_height, fc='black')

        state = np.array([quadrotor.drone_x, quadrotor.drone_y])

        #Plot lidar_readings
        if quadrotor.lidar_receive == True:
       
            iter += 1

            if len(quadrotor.lidar_readings) == 35:
                rays = np.linspace(-np.pi / 2, np.pi / 2, num_samples_laser)
                laser_x = []
                laser_y = []
                for it_laser in range(num_samples_laser):
                    laser_readings_it = quadrotor.lidar_readings[it_laser]
                    laser_intersect = state + laser_readings_it * np.array([np.cos(rays[it_laser]), np.sin(rays[it_laser])])


                    laser_x.append(laser_intersect[0])
                    laser_y.append(laser_intersect[1])


                plt.cla()
                plt.axis([-5, 5, -4, 4])
                #ax.add_artist(cylinder1)
		#ax.add_artist(cylinder2)
		#ax.add_artist(cylinder3)



                #ax.add_artist(rectangle)

                plt.scatter(laser_x, laser_y)
                ax.add_artist(drone)

                if quadrotor.action == 0:
                    ax.arrow(state[0], state[1], 0, -0.5, head_width=0.3, head_length=0.3, fc='k', ec='k')
                if quadrotor.action == 1:
                    ax.arrow(state[0], state[1], 0.5, -0.5, head_width=0.3, head_length=0.3, fc='k', ec='k')
                if quadrotor.action == 2:
                    ax.arrow(state[0], state[1], 0.5, 0, head_width=0.3, head_length=0.3, fc='k', ec='k')
                if quadrotor.action == 3:
                    ax.arrow(state[0], state[1], 0.5, 0.5, head_width=0.3, head_length=0.3, fc='k', ec='k')
                if quadrotor.action == 4:
                    ax.arrow(state[0], state[1], 0, 0.5, head_width=0.3, head_length=0.3, fc='k', ec='k')




            else:
                print("### Lost distance readings ### ({})".format(len(quadrotor.lidar_readings)))

                plt.cla()
                error = plt.Circle((quadrotor.drone_x, quadrotor.drone_y), 0.3, color='red')
                ax.add_artist(error)

            plt.show()
            plt.pause(0.0001)
            lidar_all_list.append(quadrotor.lidar_readings)
            quadrotor.lidar_receive = False
            plt.savefig('{}/lidar_readings{}.png'.format(recording_path, iter))


        if quadrotor.flight_done:
            break

        # if iter > 50:
        #     break

    lidar_all = np.array(lidar_all_list)
    with open("{}/myfile.csv".format(recording_path), 'w') as csvfile:
        Writer = csv.writer(csvfile, delimiter=',', lineterminator='\n')
        Writer.writerows(lidar_all.T)


    #quadrotor.subscriber_cb()


    print("Finish fling")



