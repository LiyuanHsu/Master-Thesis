import numpy as np
import tensorflow as tf
from HL.TrainerPolicySingleOut_LSTM import TrainerPolicySingleOut_LSTM
from HL.Architectures.LidarGoalFullyConnected import LidarGoalFullyConnected
from HL.OneHiddenFull_LSTM import OneHiddenFull_LSTM
from HL.Worker_LSTM import Worker_LSTM
import os
import gym

import threading
import multiprocessing
from time import sleep
import time

import imageio

max_episode_length = 300
gamma = .99  # discount rate for advantage estimation and reward discounting
load_model = True
record_gif = True
model_path = './model_one_step'

POLICY_PARAM = {
    'n_bootstraping': 30,  # THIS IS TEST!!! DO NOT HAVE INFLUENCE HERE!!!
}

game_name = 'PlannerComplexMap3-v0'
# game_name = 'CurEnvDivObs-v0'
# game_name = 'BigEnv1-v0'

ENV_PARMAS = {
    'seed': 0,
    'set_seed': False,
    'obstacle_radius': 0.4,
    'obstacle_num': 12,
    'obstacle_type': ['cylinder'],
    'set_obst_pose': False,
}

tf.reset_default_graph()

if not os.path.exists(model_path):
    os.makedirs(model_path)

with tf.device("/cpu:0"):
    global_episodes = tf.Variable(0, dtype=tf.int32, name='global_episodes', trainable=False)
    optimizer = tf.train.AdamOptimizer(learning_rate=1e-4) #learning_rate=1e-4
    game = gym.make(game_name)
    # game.seed(ENV_PARMAS)
    a_size = game.action_space.n
    s_size = len(game.observation_space.high)
    master_network = OneHiddenFull_LSTM(s_size, a_size, 'global')
    # master_network = LidarGoalFullyConnected(s_size, 0, a_size, 'global')
    num_workers = multiprocessing.cpu_count()  # Set workers ot number of available CPU threads
    workers = []
    policies = []
    trainers = []

    # Create worker classes
    for i in range(1):
        game = gym.make(game_name)
        # game.seed(ENV_PARMAS)
        a_size = game.action_space.n
        s_size = len(game.observation_space.high)
        policies.append(OneHiddenFull_LSTM(s_size, a_size, "worker_" + str(i)))
        # policies.append(LidarGoalFullyConnected(s_size, 0, a_size, "worker_" + str(i)))
        trainers.append(TrainerPolicySingleOut_LSTM(policies[-1], optimizer))
        workers.append(Worker_LSTM(game, i, trainers[-1], model_path, global_episodes, POLICY_PARAM))



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
    # test_game.seed(ENV_PARMAS)
    reward_sum = 0.0

    if record_gif:
        game_images = []
        actions_collect = []

        episode_num = 200
        oscillation_count = 0
        collision_count = 0
        success_count = 0

        if not os.path.exists('./gifs'):
            os.makedirs('./gifs')

        for i_episode in range(episode_num):
            observation = test_game.reset()

            rnn_state = policies[0].state_init

            reward_sum = 0
            for t in range(max_episode_length):
                image = test_game.render(mode='rgb_array')
                game_images.append(image)
                action, rnn_state = policies[0].inference(sess, observation, rnn_state)

                # action, _ = policies[0].explore(sess, observation)
                # print("action:{}".format(action))
                # print('observation size:', observation.size)
                # print('observation:', observation)
                observation, reward, done, info = test_game.step(action)
                # print(observation)
                # observation, reward, done, info = test_game.step(0)
                # print(observation)
                reward_sum += reward

                if t > 150:
                    print("###Oscillation###")
                    oscillation_count += 1
                    # imageio.mimsave('./gifs/latest-{}.gif'.format(i_episode + 1), game_images, fps=20)
                    break


                #print("reward:", reward)e
                if done:
                    print("###GOAL### Episode finished after {} timesteps".format(t + 1))
                    print(reward_sum)

                    if reward == -1:
                        collision_count += 1
                        # imageio.mimsave('./gifs/latest-{}.gif'.format(i_episode + 1), game_images, fps=20)

                    break

        # if not os.path.exists('./gifs'):
        #     os.makedirs('./gifs')
        # imageio.mimsave('./gifs/latest-{}.gif'.format(i_episode + 1), game_images, fps=20)

print('Success rate:{}'.format(1 - (oscillation_count + collision_count) / episode_num))
print("finish with {} episodes".format(episode_num))
print("oscillation times:", oscillation_count)
print("collision times:", collision_count)
print("collision ratio:", collision_count/(collision_count + oscillation_count))