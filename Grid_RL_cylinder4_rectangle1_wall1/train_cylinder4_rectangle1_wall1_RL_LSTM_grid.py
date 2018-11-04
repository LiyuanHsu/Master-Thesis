import numpy as np
import tensorflow as tf
from HL.TrainerPolicySingleOut_LSTM import TrainerPolicySingleOut_LSTM
from HL.OneHiddenFull_LSTM import OneHiddenFull_LSTM
from HL.Architectures.LidarGoalFullyConnected import LidarGoalFullyConnected
from HL.Worker_LSTM import Worker_LSTM
import os
import gym

import threading
import multiprocessing
from time import sleep
from time import time

max_episode_length = 300
gamma = .99  # discount rate for advantage estimation and reward discounting
load_model = False
model_path = './model_one_step'

POLICY_PARAM = {
    'n_bootstraping': 30,
}

game_name = 'PlannerCylinder4Rectangle1Wall1-v0'
# game_name = 'CurEnvDivObs-v0'
# game_name = 'BigEnv1-v0'

ENV_PARMAS = {
    'seed': 0,
    'set_seed': False,
    'obstacle_radius': 0.05,
    'obstacle_num': 1,
    'obstacle_type': ['cylinder'],
    'set_obst_pose': False,
}

tf.reset_default_graph()

if not os.path.exists(model_path):
    os.makedirs(model_path)

with tf.device("/cpu:0"):
    global_episodes = tf.Variable(0, dtype=tf.int32, name='global_episodes', trainable=False)
    optimizer = tf.train.AdamOptimizer(learning_rate=1e-4)
    game = gym.make(game_name)
    # game.seed(ENV_PARMAS)
    a_size = game.action_space.n
    s_size = len(game.observation_space.high)

    # Generate global network
    master_network = OneHiddenFull_LSTM(s_size, a_size, 'global')
    # master_network = LidarGoalFullyConnected(s_size, 0, a_size, 'global')
    num_workers = multiprocessing.cpu_count()  # Set workers ot number of available CPU threads
    # num_workers = 1
    workers = []
    policies = []
    trainers = []
    # Create worker classes
    for i in range(num_workers):
        game = gym.make(game_name)
        # game.seed(ENV_PARMAS)
        a_size = game.action_space.n
        s_size = len(game.observation_space.high)
        policies.append(OneHiddenFull_LSTM(s_size, a_size, "worker_" + str(i)))
        # policies.append(LidarGoalFullyConnected(s_size, 0, a_size, "worker_" + str(i)))
        trainers.append(TrainerPolicySingleOut_LSTM(policies[-1], optimizer))
        workers.append(Worker_LSTM(game, i, trainers[-1], model_path, global_episodes, POLICY_PARAM))



    saver = tf.train.Saver(max_to_keep=20)
with tf.Session() as sess:
    coord = tf.train.Coordinator()
    if load_model == True:
        print('Loading Model...')
        ckpt = tf.train.get_checkpoint_state(model_path)
        saver.restore(sess, ckpt.model_checkpoint_path)
    else:
        sess.run(tf.global_variables_initializer())

    # This is where the asynchronous magic happens.
    # Start the "work" process for each worker in a separate threat.
    worker_threads = []
    for worker in workers:
        worker_work = lambda: worker.work(max_episode_length, gamma, sess, coord, saver)
        t = threading.Thread(target=(worker_work))
        t.start()
        sleep(0.5)
        worker_threads.append(t)

    coord.join(worker_threads)



















