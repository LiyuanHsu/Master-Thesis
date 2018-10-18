

import os
import time
import numpy as np
import tensorflow as tf
import gym
import csv
import matplotlib.pyplot as plt


class Draw_matlab():
    def __init__(self, action, lidar_readings):


        #Matlab plot
        num_samples_laser = 35
        state = np.array([-0.5, 0.0])
        drone_width = 0.2
        drone_height = 0.2
        recording_path = './recording'

        if not os.path.exists(recording_path):
            os.makedirs(recording_path)




        plt.ion()
        rectangle = plt.Rectangle((state[0]-drone_width/2, state[1]-drone_height/2), drone_width, drone_height, fc='black')
        circle2 = plt.Circle((0.0, 0.0), 0.05, color='red')
        fig, ax = plt.subplots()
        x = np.arange(35)


        lidar_all_list = []
        iter = 0



        iter += 1

        rays = np.linspace(-np.pi / 2, np.pi / 2, num_samples_laser)
        laser_x = []
        laser_y = []
        for it_laser in range(num_samples_laser):
            laser_readings_it = lidar_readings[it_laser]
            laser_intersect = state + laser_readings_it * np.array([np.cos(rays[it_laser]), np.sin(rays[it_laser])])


            laser_x.append(laser_intersect[0])
            laser_y.append(laser_intersect[1])


            plt.cla()
            plt.axis([-5, 5, -5, 5])
            ax.add_artist(circle2)
            ax.add_artist(rectangle)
            plt.scatter(laser_x, laser_y)

            if action == 0:
                ax.arrow(state[0], state[1], 0, -0.5, head_width=0.3, head_length=0.3, fc='k', ec='k')
            if action == 1:
                ax.arrow(state[0], state[1], 0.5, -0.5, head_width=0.3, head_length=0.3, fc='k', ec='k')
            if action == 2:
                ax.arrow(state[0], state[1], 0.5, 0, head_width=0.3, head_length=0.3, fc='k', ec='k')
            if action == 3:
                ax.arrow(state[0], state[1], 0.5, 0.5, head_width=0.3, head_length=0.3, fc='k', ec='k')
            if action == 4:
                ax.arrow(state[0], state[1], 0, 0.5, head_width=0.3, head_length=0.3, fc='k', ec='k')



        plt.show()
        plt.pause(0.0001)


        plt.savefig('{}/lidar_readings{}.png'.format(recording_path, iter))

        lidar_all = np.array(lidar_all_list)
        with open("{}/myfile.csv".format(recording_path), 'w') as csvfile:
            Writer = csv.writer(csvfile, delimiter=',', lineterminator='\n')
            Writer.writerows(lidar_all.T)
