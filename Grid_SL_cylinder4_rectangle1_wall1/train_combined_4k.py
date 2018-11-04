import multiprocessing
import os
from time import time

import numpy as np
import pandas
import tensorflow as tf
import pandas as pd

import sys
sys.path.insert(0, '/cluster/home/lhsu/leonhard_training/')

from module.ObsLstmAct_liyuan import ObsLstmAct
from module.TrainObsLstmAct_liyuan import TrainerObsLstmAct

model_obs_2_act = './model_checkpoint'
# game_name = 'CylinderCur-v0'

POLICY_PARAM = {
    'batch_size': 20,
    'training_step_max': 1110000,
    'horizon_len': 1,
    'n_bootstraping': 60,
    'lstm_sequence': 120,
    'n_z': 2,
}

num_workers = multiprocessing.cpu_count()

if not os.path.exists(model_obs_2_act):
    os.makedirs(model_obs_2_act)

a_size = 8
s_size = 70
CSV_PATH1 = "./cylinder4_2k_grid_s1.csv"
CSV_PATH2 = "./cylinder4_2k_grid_s2.csv"
CSV_PATH3 = "./rectangle1_2k_grid_s1.csv"
CSV_PATH4 = "./rectangle1_2k_grid_s2.csv"
CSV_PATH5 = "./wall1_2k_grid_s1.csv"
CSV_PATH6 = "./wall1_2k_grid_s2.csv"





# Load csv data by tf.contrib
training_set_1 = tf.contrib.learn.datasets.base.load_csv_with_header(
    filename=CSV_PATH1,
    target_dtype=np.int,
    features_dtype=np.float32)

training_set_2 = tf.contrib.learn.datasets.base.load_csv_with_header(
    filename=CSV_PATH2,
    target_dtype=np.int,
    features_dtype=np.float32)

training_set_3 = tf.contrib.learn.datasets.base.load_csv_with_header(
    filename=CSV_PATH3,
    target_dtype=np.int,
    features_dtype=np.float32)

training_set_4 = tf.contrib.learn.datasets.base.load_csv_with_header(
    filename=CSV_PATH4,
    target_dtype=np.int,
    features_dtype=np.float32)

training_set_5 = tf.contrib.learn.datasets.base.load_csv_with_header(
    filename=CSV_PATH5,
    target_dtype=np.int,
    features_dtype=np.float32)

training_set_6 = tf.contrib.learn.datasets.base.load_csv_with_header(
    filename=CSV_PATH6,
    target_dtype=np.int,
    features_dtype=np.float32)


training_set_1.data[np.isnan(training_set_1.data)] = 10
training_set_2.data[np.isnan(training_set_2.data)] = 10
training_set_3.data[np.isnan(training_set_3.data)] = 10
training_set_4.data[np.isnan(training_set_4.data)] = 10
training_set_5.data[np.isnan(training_set_5.data)] = 10
training_set_6.data[np.isnan(training_set_6.data)] = 10




# data_samples_num = int(training_set.data[-1, 0] + 1)

data_samples_num = 12000
data_num_per_csv = 2000
print('data_samples_num:', data_samples_num)

action_samples_in = np.zeros((data_samples_num, POLICY_PARAM['lstm_sequence'], POLICY_PARAM['horizon_len']), dtype=int)
action_samples_out = np.zeros((data_samples_num, POLICY_PARAM['lstm_sequence'], POLICY_PARAM['horizon_len']), dtype=int)
observation_samples = np.zeros((data_samples_num, POLICY_PARAM['lstm_sequence'], s_size + 2))
sequence_len = np.zeros(data_samples_num)

# counter = 0
# for it_path in range(data_samples_num):
#     data_len = np.size(training_set.data[training_set.data[:, 0] == it_path], 0)
#     if data_len < 30:
#         print('it_path:%d   data_len:%d   counter:%d' % (it_path, data_len, counter))
#         counter += 1

sample_count = 0
for sample in (training_set_1, training_set_2, training_set_3, training_set_4, training_set_5, training_set_6):
    sum_data_len = 0
    for it_path in range(data_num_per_csv):
        data_len = np.size(sample.data[sample.data[:, 0] == it_path], 0)
        sequence_len[it_path + (data_num_per_csv*sample_count)] = data_len
        if data_len > 70:
            print('it_path:{} data_len:{}'.format(it_path, data_len))
            # print('data_len:', data_len)
            # pass
        observation_samples[(it_path + (data_num_per_csv*sample_count)), 0:data_len, :] = sample.data[sample.data[:, 0] == it_path][:, 1:]
     # training_set_target = np.expand_dims(training_set.target, axis=1)
        action_samples_out[(it_path + (data_num_per_csv*sample_count)), 0:data_len, 0] = sample.target[sum_data_len:sum_data_len + data_len]

        sum_data_len += data_len
    sample_count += 1

# for it_path in range(200):
#     for sequence in range(sequence_len[it_path].astype(np.int32)):
#         for action_horizon in range(5):
#             action_horizon += 1
#             try:
#                 action_samples_out[it_path, sequence, action_horizon] = action_samples_out[it_path-1, sequence + action_horizon, 0]
#             except:
#                 pass



train_data = {'action_out': action_samples_out.astype(np.int32),
              'observation_samples': observation_samples,
              'sequence_len': sequence_len.astype(np.int32)}

# print('sequence:', train_data['sequence_len'][10])
# print(train_data['action_out'][10])
#
# print('observation_samples:', train_data['sequence_len'])


tf.reset_default_graph()

optimizer = tf.train.AdamOptimizer(learning_rate=1e-4)
policy_horizon_obs = ObsLstmAct(s_size+2, a_size, "policy_pred_imitation_vae_obs", POLICY_PARAM)
trainer_imitation = TrainerObsLstmAct(policy_horizon_obs, optimizer)

with tf.Session() as sess:
    sess.run(tf.global_variables_initializer())
    trainer_imitation.train(sess, train_data, model_obs_2_act, POLICY_PARAM)


print ('finish')
