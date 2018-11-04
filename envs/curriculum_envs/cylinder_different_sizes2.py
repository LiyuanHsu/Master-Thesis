import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import numpy.matlib


class CylinderDiffSizesEnv2(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self):
        self.goal_pos = np.array([0.0, -3.0])
        self.start_pos = np.array([-14.0, 0.0])
        self.bound_box = np.array([[-15.0, 2.0], [-4.0, 4.0]])

        self.state = self.start_pos
        self.viewer = None

        self.num_samples_laser = 140
        # self.max_measurement_laser = np.random.random_integers(5)
        self.max_measurement_laser = 7

        self.laser_obs = np.random.random_integers(10) * np.ones(self.num_samples_laser)
        # self.laser_obs = 100.0 * np.ones(self.num_samples_laser)

        self.lidar_readings = np.ones(35)

        down_obs = np.zeros(self.num_samples_laser)
        up_obs = 100*np.ones(self.num_samples_laser)

        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(down_obs, up_obs)

        self.obstacle_num = 1

        # self.obstacle_radius = np.array([0.8])

        self.obstacle_radius = np.array([0.8])
        # self.obstacle_radius = np.array([0.2, 0.4, 0.8, 1.0])
        self.objects_pos = np.zeros((self.obstacle_num, 2))

        self.objects_pos = np.array([-5.5, 0.5])
        # self.objects_pos = np.array([[-5.5, 0.5], [-7.0, 1.0], [-3.0, 1.5], [-4.5, -1.3]])

    def set_obstacle_radius(self, radius):
        self.obstacle_radius = radius

    def _seed(self, seed=None): # this is a hack, but no easy way to set parameters
        #self.set_obstacle_radius(seed['obstacle_radius'])
        #self.obstacle_num = seed['obstacle_num']
        #self.objects_pos = np.zeros((self.obstacle_num, 2))
        if seed['set_seed']:
            self.np_random, seed = seeding.np_random(seed['seed'])
        return [True]

    def _step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        state = self.state
        old_state = self.state

        d_state = self.action_2_state_d(action)

        self.state = state + d_state
        done = self.state[0] < self.bound_box[0, 0] \
               or self.state[0] > self.bound_box[0, 1] \
               or self.state[1] < self.bound_box[1, 0] \
               or self.state[1] > self.bound_box[1, 1]

        done = bool(done)

        reward = 0.0
        if done:
            reward = -1.0
        else:
            for it_obstacles in range(self.obstacle_num):
                if np.linalg.norm(self.state - self.objects_pos) \
                        < (self.obstacle_radius[it_obstacles] + 0.4):
                    reward = -1.0
                    done = True
                    break

            if not done:
                if self.state[0] > 0.0:
                    reward = 1.0
                    done = True
                else:
                    if np.linalg.norm(self.state - self.goal_pos) < np.linalg.norm(old_state - self.goal_pos):
                        reward = 0.01

        observation = self.laser_readings()
        # print(self.laser_readings())

        self.laser_obs = np.copy(observation)
        distance_vec = self.goal_pos - self.state

        # measurement = np.concatenate((observation, distance_vec))
        measurement = observation

        info = self.objects_pos


        return measurement, reward, done, info

    def action_2_state_d(self, action):
        return {
            0: np.array([0.0, -0.5]),
            1: np.array([0.353, -0.353]),
            2: np.array([0.5, 0.0]),
            3: np.array([0.353, 0.353]),
            4: np.array([0.0, 0.5]),
        }[action]

    def _reset(self):

        self.state = self.start_pos

        self.objects_pos = np.array([-5.5, 0.5])
        # self.objects_pos = np.array([[-5.5, 0.5], [-7.0, 1.0], [-3.0, 1.5], [-4.5, -1.3]])

        self.goal_pos[0] = 0.0
        self.goal_pos[1] = 0.0

        observation = self.laser_readings()
        self.laser_obs = np.copy(observation)
        distance_vec = self.goal_pos - self.state

        #return np.concatenate((observation, distance_vec))
        return observation

    # def _get_lidar(self, lidar):
    #     self.lidar = lidar

    def _render(self, mode='human', close=False):
        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        screen_width = 1000
        screen_height = 600

        scale_width = screen_width / (self.bound_box[0, 1] - self.bound_box[0, 0])
        scale_height = screen_height / (self.bound_box[1, 1] - self.bound_box[1, 0])

        zero_width = scale_width * (-self.bound_box[0, 0])
        zero_height = scale_height * (-self.bound_box[1, 0])

        drone_width = 20
        drone_height = 20

        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(screen_width, screen_height)

            l, r, t, b = -drone_width / 2, drone_width / 2, drone_height / 2, -drone_height / 2
            drone = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            self.drone_trans = rendering.Transform()
            drone.add_attr(self.drone_trans)
            self.viewer.add_geom(drone)

            goal = rendering.make_circle(screen_height / 6.0 * 0.5)
            goal.set_color(.0, 1.0, .0)
            self.goal_trans = rendering.Transform()
            goal.add_attr(self.goal_trans)
            self.viewer.add_geom(goal)

            obstacles = []
            self.obstacle_trans = []
            for it_obstacles in range(self.obstacle_num):
                radius = screen_height / 8.0 * self.obstacle_radius[it_obstacles]
                obstacles.append(rendering.make_circle(radius))
                obstacles[it_obstacles].set_color(1.0, .0, .0)
                self.obstacle_trans.append(rendering.Transform())
                obstacles[it_obstacles].add_attr(self.obstacle_trans[it_obstacles])
                self.viewer.add_geom(obstacles[it_obstacles])

            laser_readings = []
            self.laser_readings_array = []
            for it_laser in range(self.num_samples_laser):
                laser_readings.append(rendering.make_circle(3))
                laser_readings[it_laser].set_color(0.0, 0.0, 1.0)
                self.laser_readings_array.append(rendering.Transform())
                laser_readings[it_laser].add_attr(self.laser_readings_array[it_laser])
                self.viewer.add_geom(laser_readings[it_laser])
                # print('type:', type(laser_readings))
                # print('size:', len(laser_readings))
                # print('laser_readings', observation)
                # print('Run viewer')


        if self.state is None: return None


        counter = 0

        drone_x = self.state[0] * scale_width + zero_width
        drone_y = self.state[1] * scale_height + zero_height
        self.drone_trans.set_translation(drone_x, drone_y)

        # goal_x = np.random.random_integers(3) * scale_width + zero_width
        goal_x = self.goal_pos[0] * scale_width + zero_width
        # goal_y =  np.random.random_integers(3) * scale_height + zero_height
        goal_y = self.goal_pos[1] * scale_height + zero_height
        self.goal_trans.set_translation(goal_x, goal_y)

        if self.obstacle_num == 1:
            object_it_pos = self.objects_pos
            obstacle_x = object_it_pos[0] * scale_width + zero_width
            obstacle_y = object_it_pos[1] * scale_height + zero_height
            self.obstacle_trans[0].set_translation(obstacle_x, obstacle_y)
        else:
            for it_obstacles in range(self.obstacle_num):
                object_it_pos = self.objects_pos[it_obstacles, :]
                obstacle_x = object_it_pos[0] * scale_width + zero_width
                obstacle_y = object_it_pos[1] * scale_height + zero_height
                self.obstacle_trans[it_obstacles].set_translation(obstacle_x, obstacle_y)

        if True:
            counter = counter + 1
            rays = np.linspace(-np.pi / 2, np.pi / 2, self.num_samples_laser)
            for it_laser in range(self.num_samples_laser):
                # print(self.lidar)
                # self.laser_obs = np.random.random_integers(5) * np.ones(self.num_samples_laser)
                laser_reading_it = self.laser_obs[it_laser]
                print(self.laser_obs)

                # if laser_reading_it > np.random.random_integers(5):
                #     laser_reading_it = np.random.random_integers(5)

                if laser_reading_it > self.max_measurement_laser:
                    laser_reading_it = self.max_measurement_laser



                laser_intersect = self.state[:2] + laser_reading_it * \
                                  np.array([np.cos(rays[it_laser]),
                                            np.sin(rays[it_laser])])


                # laser_x = -np.random.random_integers(13) * scale_width + zero_width
                laser_x = laser_intersect[0] * scale_width + zero_width
                laser_y = laser_intersect[1] * scale_height + zero_height
                self.laser_readings_array[it_laser].set_translation(laser_x, laser_y)
                # print('type: ',type(self.laser_obs))
                # print('size: ',self.laser_obs.size)
                # print(self.laser_obs)
                # print('laser_reading_it', laser_reading_it)




        return self.viewer.render(return_rgb_array=mode == 'rgb_array')




    def laser_readings(self):

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        lidar_readings = np.random.random_integers(10) * np.ones(num_samples)

        if self.obstacle_num == 1:
            if np.linalg.norm(self.state - self.objects_pos) <= max_measurement:
                single_obstacle_readings = self.laser_reading_single_cylinder(self.objects_pos, self.obstacle_radius)
                lidar_readings[single_obstacle_readings < lidar_readings] = single_obstacle_readings[
                    single_obstacle_readings < lidar_readings]
        else:
            for it in range(self.obstacle_num):
                if np.linalg.norm(self.state - self.objects_pos[it, :]) <= max_measurement:
                    single_obstacle_readings = \
                        self.laser_reading_single_cylinder(self.objects_pos[it, :], self.obstacle_radius[it])
                    lidar_readings[single_obstacle_readings < lidar_readings] = single_obstacle_readings[
                        single_obstacle_readings < lidar_readings]

        return lidar_readings

    def laser_reading_single_cylinder(self, obstacle, radius_obs):

        circle_center = obstacle

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        rays = np.linspace(-np.pi / 2, np.pi / 2, num_samples)
        directions = np.array([np.cos(rays), np.sin(rays)])
        radius = radius_obs

        t0 = np.zeros(num_samples)

        quad_pose = self.state

        A = np.sum(directions ** 2, axis=0)

        # print('circle_center shape', circle_center.shape)
        # print('circle_center type', type(circle_center))
        # print(circle_center)
        #
        #
        # print('quad_pose shape', quad_pose.shape)
        # print('quad_pose type', type(quad_pose))
        # print(quad_pose)
        #
        # print(quad_pose - circle_center)
        #
        #
        #
        # print('np.transpose(directions) shape', np.transpose(directions).shape)
        # print('np.transpose(directions) type', type(np.transpose(directions)))
        # print(np.transpose(directions))
        #
        #
        #
        # print('np.multiply shape', np.multiply(np.transpose(directions), quad_pose - circle_center).shape)


        B = 2.0 * np.sum(np.multiply(np.transpose(directions), quad_pose - circle_center), axis=1)
        C = np.sum((quad_pose - circle_center) ** 2.0, axis=0) - radius ** 2.0


        mid_result = B ** 2.0 - 4.0 * C * A

        zero_array = np.zeros(np.shape(mid_result))
        less_zero = np.less(mid_result, zero_array)
        greater_zero = np.logical_not(less_zero)

        t0[less_zero] = np.inf

        mid_result_2 = mid_result[greater_zero] ** (0.5)
        t0[greater_zero] = (-B[greater_zero] - mid_result_2) / (2.0 * A[greater_zero])

        negative_t0 = t0 < 0
        t0[negative_t0] = np.inf

        intersection_distace = t0
        intersection_distace[t0 > max_measurement] = 50

        return intersection_distace

    # def laser_readings(self):
    #
    #     num_samples = self.num_samples_laser
    #     max_measurement = 3
    #
    #     rays = np.linspace(-np.pi / 2, np.pi / 2, num_samples)
    #     directions = np.array([np.cos(rays), np.sin(rays)])
    #     radius = self.obstacle_radius
    #
    #     t0 = np.zeros((num_samples))
    #
    #     circle_center = self.objects_pos
    #     quad_pose = self.state
    #
    #     A = np.sum(directions ** 2, axis=0)
    #     B = 2.0 * np.sum(np.multiply(np.transpose(directions), quad_pose - circle_center), axis=1)
    #     C = np.sum((quad_pose - circle_center) ** 2.0, axis=0) - radius ** 2.0
    #
    #     mid_result = B ** 2.0 - 4.0 * C * A
    #
    #     zero_array = np.zeros(np.shape(mid_result))
    #     less_zero = np.less(mid_result, zero_array)
    #     greater_zero = np.logical_not(less_zero)
    #
    #     t0[less_zero] = np.inf
    #
    #     mid_result_2 = mid_result[greater_zero] ** (0.5)
    #     t0[greater_zero] = (-B[greater_zero] - mid_result_2) / (2.0 * A[greater_zero])
    #
    #     negative_t0 = t0 < 0
    #     t0[negative_t0] = np.inf
    #
    #     intersection_distace = t0
    #     intersection_distace[t0 > max_measurement] = 100
    #
    #     return intersection_distace
    def lidar(self,lidar_readings):
        self.lidar_readings = lidar_readings
