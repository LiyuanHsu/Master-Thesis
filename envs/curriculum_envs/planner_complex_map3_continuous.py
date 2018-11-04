import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import numpy.matlib
import math


class PlannerComplexMap3_continuous(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self):
        self.goal_pos = np.array([5.0, 5.0])
        self.start_pos = np.array([2.0, 2.0, 0.0])
        self.bound_box = np.array([[-1.0, 10.0], [-1.0, 10.0]])

        self.state = np.copy(self.start_pos)
        self.viewer = None

        self.num_samples_laser = 70
        self.max_measurement_laser = 10.0
        self.laser_obs = 100.0 * np.ones(self.num_samples_laser)

        down_obs = np.zeros(self.num_samples_laser)
        up_obs = 100*np.ones(self.num_samples_laser)

        # self.action_space = spaces.Discrete(8*3)
        # self.action_space = spaces.Discrete(5 * 3)
        self.action_space = spaces.Discrete(8)
        # self.action_space = spaces.Discrete(7)
        self.observation_space = spaces.Box(down_obs, up_obs)
        self.manual_pose = False
        self.goal_radius = 0.2
        self.drone_radius = 0.1

        self.obstacle_num = 12
        self.obstacle_radius = 1
        self.obstacle_type = ['rectangle', 'rectangle', 'rectangle', 'rectangle', 'rectangle', 'rectangle', 'rectangle', 'rectangle','rectangle', 'rectangle', 'cylinder', 'cylinder']
        self.obstacle_pos = np.zeros((self.obstacle_num, 3))
        self.obstacle_dim = []
        self.obstacle_dim.append(self.obstacle_radius)

    def set_obstacle_radius(self, radius):
        self.obstacle_radius = radius

    def _seed(self, seed=None): # this is a hack, but no easy way to set parameters
        self.set_obstacle_radius(seed['obstacle_radius'])
        self.obstacle_num = seed['obstacle_num']
        self.obstacle_pos = np.zeros((self.obstacle_num, 3))
        self.obstacle_type = seed['obstacle_type']
        if seed['set_seed']:
            self.np_random, seed = seeding.np_random(seed['seed'])
        if seed['set_obst_pose']:
            self.manual_pose = True
            self.obstacle_pos[:, 0] = seed['obst_pose'][0]
            self.obstacle_pos[:, 1] = seed['obst_pose'][1]
            self.goal_pos[0] = seed['goal_pose'][0]
            self.goal_pos[1] = seed['goal_pose'][1]

        return [True]

    def _step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        old_state = np.copy(self.state)

        d_state = self.action_2_state_d(action)

        rot_mat = np.array([[np.cos(self.state[2]), -np.sin(self.state[2])],
                            [np.sin(self.state[2]), np.cos(self.state[2])]])
        d_state_trans = np.matmul(rot_mat, d_state[0:2])

        self.state[0:2] = self.state[0:2] + d_state_trans
        self.state[2] = self.state[2] + d_state[2]

        self.state[2] = (self.state[2] + np.pi) % (2 * np.pi) - np.pi

        done = self.state[0] < self.bound_box[0, 0] \
               or self.state[0] > self.bound_box[0, 1] \
               or self.state[1] < self.bound_box[1, 0] \
               or self.state[1] > self.bound_box[1, 1]

        done = bool(done)

        reward = 0.0
        if done:
            # reward = -1.0  # TODO: maybe not to penalize for going out
            reward = 0.0
        else:
            collision = self.test_collision(self.state[:2], self.drone_radius)

            if collision:
                reward = -1.0
                done = True
            else:
                if np.linalg.norm(self.goal_pos - self.state[0:2]) < 0.6:
                    reward = 1.0
                    done = True
                else:
                    if np.linalg.norm(self.state[0:2] - self.goal_pos) < np.linalg.norm(old_state[0:2] - self.goal_pos):
                        #reward = 0.0005
                        # reward = 0.01
                        #reward = 0.001
                        pass

        observation = self.laser_readings()
        self.laser_obs = np.copy(observation)
        distance_vec = self.goal_pos - self.state[0:2]
        distance = np.linalg.norm(distance_vec)
        angle = np.arctan2(distance_vec[1], distance_vec[0])
        angle = angle - self.state[2]
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        goal_measurements = np.array([distance, angle])

        measurement = np.concatenate((observation, goal_measurements)) # After (feed observation first)
        # measurement = np.concatenate((goal_measurements, observation))  # Before (feed goal_measurement first)
        # measurement = goal_measurements

        info = self.obstacle_pos

        return measurement, reward, done, info


    # def action_2_state_d(self, action):
    #     factor = 1
    #     return {
    #         0: np.multiply([0.2, 0.0, 0.0], factor),
    #         1: np.multiply([0.2, 0.2, 0.0], factor),
    #         2: np.multiply([0.0, 0.2, 0.0], factor),
    #         3: np.multiply([-0.2, 0.2, 0.0], factor),
    #         4: np.multiply([-0.2, 0.0, 0.0], factor),
    #         5: np.multiply([-0.2, -0.2, 0.0], factor),
    #         6: np.multiply([0.0, -0.2, 0.0], factor),
    #         7: np.multiply([0.2, -0.2, 0.0], factor),
    #     }[action]

    def action_2_state_d(self, action):
        factor = 1
        step_size = 0.4
        angle_inc = 22.5

        return {
            0: np.multiply([0.2, 0.0, 0.0], factor),
            1: np.multiply([0.1414, 0.1414, 0.0], factor),
            2: np.multiply([0.0, 0.2, 0.0], factor),
            3: np.multiply([-0.1414, 0.1414, 0.0], factor),
            4: np.multiply([-0.2, 0.0, 0.0], factor),
            5: np.multiply([-0.1414, -0.1414, 0.0], factor),
            6: np.multiply([0.0, -0.2, 0.0], factor),
            7: np.multiply([0.1414, -0.1414, 0.0], factor),
        }[action]


    def _reset(self):

        self.state = np.copy(self.start_pos)

        if not self.manual_pose:


            # obstacle pose
            self.obstacle_dim = []

            for it_obstacle in range(self.obstacle_num):
                if self.obstacle_type[it_obstacle] == 'cylinder':
                    self.obstacle_pos[it_obstacle, 2] = 0.0

                    # for i in range (50):
                    #     self.obstacle_pos[it_obstacle, 0] = np.random.uniform(1, 7, 1)
                    #     self.obstacle_pos[it_obstacle, 1] = np.random.uniform(1, 7, 1)
                    #
                    #     if np.linalg.norm(self.obstacle_pos[it_obstacle, :2] - self.goal_pos) > 0.6 and np.linalg.norm(self.obstacle_pos[it_obstacle, :2] - self.state[:2]) > 1.0:
                    #         break

                    # self.obstacle_pos[it_obstacle, 0] = 4
                    # self.obstacle_pos[it_obstacle, 1] = 1.5
                    #


                    self.obstacle_pos[10, 0:2] = [1.5, 7.5]
                    self.obstacle_pos[11, 0:2] = [7.5, 1.5]


                    radius = 0.4
                    self.obstacle_dim.append(radius)

                elif self.obstacle_type[it_obstacle] == 'rectangle':
                    obstacle_pose_direction = np.random.uniform(1.5, 4.5, 1)
                    obstacle_pose_perpendicular = np.random.uniform(1.0, 7.0, 1)

                    # obstacle_x = obstacle_pose_direction * np.cos(angle_goal) - obstacle_pose_perpendicular * np.sin(
                    #     angle_goal)
                    # obstacle_y = obstacle_pose_direction * np.sin(angle_goal) + obstacle_pose_perpendicular * np.cos(
                    #     angle_goal)

                    obstacle_x = 0
                    obstacle_y = 4.5

                    self.obstacle_pos[it_obstacle, 0] = obstacle_x
                    self.obstacle_pos[it_obstacle, 1] = obstacle_y

                    self.obstacle_pos[0, 0] = 0
                    self.obstacle_pos[0, 1] = 4.5

                    self.obstacle_pos[1, 0] = 9
                    self.obstacle_pos[1, 1] = 4.5

                    self.obstacle_pos[2, 0] = 4.5
                    self.obstacle_pos[2, 1] = 9

                    self.obstacle_pos[3, 0] = 4.5
                    self.obstacle_pos[3, 1] = 0

                    self.obstacle_pos[4, 0] = 4.5
                    self.obstacle_pos[4, 1] = 4.5

                    self.obstacle_pos[5, 0] = 2.5
                    self.obstacle_pos[5, 1] = 5

                    self.obstacle_pos[6, 0] = 6.5
                    self.obstacle_pos[6, 1] = 4

                    self.obstacle_pos[7, 0] = 2
                    self.obstacle_pos[7, 1] = 0.75

                    self.obstacle_pos[8, 0] = 4
                    self.obstacle_pos[8, 1] = 8.5

                    self.obstacle_pos[9, 0] = 8
                    self.obstacle_pos[9, 1] = 7



                    # self.obstacle_pos[it_obstacle, 2] = np.random.uniform(-np.pi, np.pi, 1)
                    self.obstacle_pos[it_obstacle, 2] = 0

                    # dimensions = np.array([4.5, 0.2])
                    # self.obstacle_dim.append(dimensions)

                    # self.obstacle_dim = [np.array([0.2, 9.2]), np.array([0.2, 9.2]), np.array([9.2, 0.2]), np.array([9.2, 0.2]), np.array([0.2, 2]), np.array([3.2, 0.2]), np.array([3, 0.2]), np.array([0.2, 3]), np.array([3, 0.2])]

                    # Without dead-end
                    self.obstacle_dim = [np.array([0.2, 9.2]), np.array([0.2, 9.2]), np.array([9.2, 0.2]), np.array([9.2, 0.2]), np.array([3, 3]), np.array([1, 0.5]), np.array([1, 0.5]), np.array([0.5, 1.5]), np.array([0.5, 1]), np.array([2, 0.4])]


                elif self.obstacle_type[it_obstacle] == 'wall':

                    # this is for perpendicular wall TODO: add horizontal wall
                    obstacle_pose_direction = np.random.uniform(1.5, 4.5, 1)
                    obstacle_pose_perpendicular = np.random.uniform(1, 7.0, 1)

                    # obstacle_x = obstacle_pose_direction * np.cos(angle_goal) - obstacle_pose_perpendicular * np.sin(angle_goal)
                    # obstacle_y = obstacle_pose_direction * np.sin(angle_goal) + obstacle_pose_perpendicular * np.cos(angle_goal)

                    obstacle_x = np.random.uniform(1, 7, 1)
                    obstacle_y = np.random.uniform(1, 7, 1)

                    self.obstacle_pos[it_obstacle, 0] = obstacle_x
                    self.obstacle_pos[it_obstacle, 1] = obstacle_y
                    self.obstacle_pos[it_obstacle, 2] = 0.0

                    wall_second_point = np.array([2.0, 2.0])

                    obstacle_pose_direction = np.random.uniform(1.5, 4.5, 1)
                    obstacle_pose_perpendicular = np.random.uniform(2.0, 3.0, 1)

                    # obstacle_x = obstacle_pose_direction * np.cos(angle_goal) - obstacle_pose_perpendicular * np.sin(
                    #     angle_goal)
                    # obstacle_y = obstacle_pose_direction * np.sin(angle_goal) + obstacle_pose_perpendicular * np.cos(
                    #     angle_goal)

                    wall_second_point[0] = obstacle_x
                    wall_second_point[1] = obstacle_y

                    self.obstacle_dim.append(wall_second_point)

            # TODO: can check collision at start of goal and obstacle


            # Goal pose
            angle_goal = np.pi

            for i in range(100000):
                goal_x = np.random.uniform(1, 8, 1)
                goal_y = np.random.uniform(1, 8, 1)


                # Rand to 0.5
                # self.goal_pos[0] = math.floor(goal_x * 2 + 0.5) / 2
                # self.goal_pos[1] = math.floor(goal_y * 2 + 0.5) / 2

                # Rand to 0.2
                self.goal_pos[0] = math.floor(goal_x * 5 + 0.5) / 5
                self.goal_pos[1] = math.floor(goal_y * 5 + 0.5) / 5

                if self.test_collision(self.goal_pos, 0.3):
                    continue
                else:
                    break


            # Start pose
            for i in range(100000):
                state_x = np.random.uniform(1, 8, 1)
                state_y = np.random.uniform(1, 8, 1)

                # Rand to 0.5
                # self.state[0] = math.floor(state_x * 2 + 0.5) / 2
                # self.state[1] = math.floor(state_y * 2 + 0.5) / 2

                # Rand to 0.2
                self.state[0] = math.floor(state_x * 5 + 0.5) / 5
                self.state[1] = math.floor(state_y * 5 + 0.5) / 5

                if np.linalg.norm(self.state[:2] - self.goal_pos) < 2.0 or self.test_collision(self.state[:2], 0.3):
                    continue
                else:
                    break


        observation = self.laser_readings()
        self.laser_obs = np.copy(observation)
        distance_vec = self.goal_pos - self.state[0:2]
        distance = np.linalg.norm(distance_vec)
        angle = np.arctan2(distance_vec[1], distance_vec[0])
        angle = angle - self.state[2]
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        goal_measurements = np.array([distance, angle])

        # return np.concatenate((goal_measurements, observation)) # Before
        return np.concatenate((observation, goal_measurements)) # After

        # return goal_measurements

    def _render(self, mode='human', close=False):
        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        screen_width = 800
        screen_height = 800

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

            goal = rendering.make_circle(scale_height * 0.2)
            goal.set_color(.0, 1.0, .0)
            self.goal_trans = rendering.Transform()
            goal.add_attr(self.goal_trans)
            self.viewer.add_geom(goal)

            obstacles = []
            self.obstacle_trans = []

            # add_geom
            for it_obstacle in range(self.obstacle_num):
                if self.obstacle_type[it_obstacle] == 'cylinder':
                    radius = scale_height * self.obstacle_dim[it_obstacle]
                    obstacles.append(rendering.make_circle(radius))
                    obstacles[it_obstacle].set_color(1.0, .0, .0)
                    self.obstacle_trans.append(rendering.Transform())
                    obstacles[it_obstacle].add_attr(self.obstacle_trans[it_obstacle])
                    self.viewer.add_geom(obstacles[it_obstacle])

                elif self.obstacle_type[it_obstacle] == 'rectangle':
                    obst_dim = self.obstacle_dim[it_obstacle]
                    rect_width = obst_dim[0] * scale_width
                    rect_height = obst_dim[1] * scale_height
                    l, r, t, b = -rect_width / 2, rect_width / 2, rect_height / 2, -rect_height / 2
                    obstacles.append(rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)]))
                    obstacles[it_obstacle].set_color(1.0, .0, .0)
                    self.obstacle_trans.append(rendering.Transform())
                    obstacles[it_obstacle].add_attr(self.obstacle_trans[it_obstacle])
                    self.viewer.add_geom(obstacles[it_obstacle])

                elif self.obstacle_type[it_obstacle] == 'wall':
                    wall_vector = self.obstacle_dim[it_obstacle] - self.obstacle_pos[it_obstacle, :2]
                    wall_width = np.linalg.norm(wall_vector) * scale_width
                    wall_height = 5
                    l, r, t, b = 0, wall_width, 0, wall_height
                    obstacles.append(rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)]))
                    obstacles[it_obstacle].set_color(1.0, 0.0, 0.0)
                    self.obstacle_trans.append(rendering.Transform())
                    obstacles[it_obstacle].add_attr(self.obstacle_trans[it_obstacle])
                    self.viewer.add_geom(obstacles[it_obstacle])

            laser_readings = []
            self.laser_readings_array = []
            for it_laser in range(self.num_samples_laser):
                laser_readings.append(rendering.make_circle(3))
                laser_readings[it_laser].set_color(0.0, 0.0, 1.0)
                self.laser_readings_array.append(rendering.Transform())
                laser_readings[it_laser].add_attr(self.laser_readings_array[it_laser])
                self.viewer.add_geom(laser_readings[it_laser])

        if self.state is None: return None

        drone_x = self.state[0] * scale_width + zero_width
        drone_y = self.state[1] * scale_height + zero_height
        self.drone_trans.set_translation(drone_x, drone_y)
        self.drone_trans.set_rotation(self.state[2])

        goal_x = self.goal_pos[0] * scale_width + zero_width
        goal_y = self.goal_pos[1] * scale_height + zero_height
        self.goal_trans.set_translation(goal_x, goal_y)



        # set_translation
        for it_obstacles in range(self.obstacle_num):
            if self.obstacle_type[it_obstacles] == 'cylinder':
                object_it_pos = self.obstacle_pos[it_obstacles]
                obstacle_x = object_it_pos[0] * scale_width + zero_width
                obstacle_y = object_it_pos[1] * scale_height + zero_height
                self.obstacle_trans[it_obstacles].set_translation(obstacle_x, obstacle_y)

            elif self.obstacle_type[it_obstacles] == 'rectangle':
                object_it_pos = self.obstacle_pos[it_obstacles]
                obstacle_x = object_it_pos[0] * scale_width + zero_width
                obstacle_y = object_it_pos[1] * scale_height + zero_height
                self.obstacle_trans[it_obstacles].set_translation(obstacle_x, obstacle_y)
                self.obstacle_trans[it_obstacles].set_rotation(object_it_pos[2])

            elif self.obstacle_type[it_obstacles] == 'wall':
                wall_pose = self.obstacle_pos[it_obstacles]
                wall_x = wall_pose[0] * scale_width + zero_width
                wall_y = wall_pose[1] * scale_height + zero_height
                self.obstacle_trans[it_obstacles].set_translation(wall_x, wall_y)

                wall_vector = self.obstacle_dim[it_obstacles] - wall_pose[:2]
                rotation_angle = np.arctan2(wall_vector[1], wall_vector[0])
                self.obstacle_trans[it_obstacles].set_rotation(rotation_angle)

        if True:
            rays = np.linspace(-np.pi, np.pi, self.num_samples_laser)
            for it_laser in range(self.num_samples_laser):
                laser_reading_it = self.laser_obs[it_laser]
                if laser_reading_it > self.max_measurement_laser:
                    laser_reading_it = self.max_measurement_laser
                laser_intersect = self.state[:2] + laser_reading_it *\
                                                   np.array([np.cos(self.state[2] + rays[it_laser]),
                                                             np.sin(self.state[2] + rays[it_laser])])
                laser_x = laser_intersect[0] * scale_width + zero_width
                laser_y = laser_intersect[1] * scale_height + zero_height
                self.laser_readings_array[it_laser].set_translation(laser_x, laser_y)

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

    def laser_readings(self):

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        lidar_readings = 100 * np.ones(num_samples)

        for it_obstacle in range(self.obstacle_num):
            if self.obstacle_type[it_obstacle] == 'cylinder':
                single_obstacle_readings = \
                    self.laser_reading_single_cylinder(self.obstacle_pos[it_obstacle, :2],
                                                       self.obstacle_dim[it_obstacle])
            elif self.obstacle_type[it_obstacle] == 'rectangle':
                single_obstacle_readings = self.laser_readings_rectangle(self.obstacle_pos[it_obstacle],
                                                                          self.obstacle_dim[it_obstacle])
            elif self.obstacle_type[it_obstacle] == 'wall':
                wall_points = np.zeros((2, 2))
                wall_points[0, :] = self.obstacle_pos[it_obstacle, :2]
                wall_points[1, :] = self.obstacle_dim[it_obstacle]
                single_obstacle_readings = self.laser_intersect_wall(wall_points)

            lidar_readings[single_obstacle_readings < lidar_readings] = single_obstacle_readings[
                single_obstacle_readings < lidar_readings]


        # for it in range(self.obstacle_num):
        #     if np.linalg.norm(self.state[0:2] - self.obstacle_pos[it, :]) <= max_measurement:
        #         single_obstacle_readings = self.laser_reading_single_cylinder(self.obstacle_pos[it, :])
        #         lidar_readings[single_obstacle_readings < lidar_readings] = single_obstacle_readings[
        #             single_obstacle_readings < lidar_readings]
        return lidar_readings

    def laser_reading_single_cylinder(self, obstacle, radius_obstacle):

        circle_center = obstacle

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        rays = np.linspace(self.state[2] - np.pi, self.state[2] + np.pi, num_samples)
        directions = np.array([np.cos(rays), np.sin(rays)])
        radius = radius_obstacle

        t0 = np.zeros(num_samples)

        quad_pose = self.state[0:2]

        A = np.sum(directions ** 2, axis=0)
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

        intersection_distance = t0
        intersection_distance[t0 > max_measurement] = 100


        return intersection_distance

    def laser_intersect_wall(self, wall_points):

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        laser_readings = np.inf*np.ones(num_samples)
        quad_pose = self.state[0:2]
        quad_angle = self.state[2]

        rays = np.linspace(quad_angle - np.pi, quad_angle + np.pi, num_samples)
        directions = np.transpose(np.array([np.cos(rays), np.sin(rays)]))

        wall_vector = wall_points[1, :] - wall_points[0, :]
        wall_start = wall_points[0, :]

        denum = np.cross(directions, wall_vector)

        # determine if lines are parallel
        not_zero = np.logical_not(np.equal(denum, 0.0))

        wall_intersection = np.cross(wall_start - quad_pose, directions[not_zero, :])/denum[not_zero]

        # find intersections in range 0 to 1
        wall_intersection_less_zero = wall_intersection > 0.0
        wall_intersection_grater_zero = wall_intersection < 1.0
        intersect_ind = np.logical_and(wall_intersection_less_zero, wall_intersection_grater_zero)

        # rays that intersect
        rays_intersecting = False*np.ones(num_samples, dtype=bool)
        rays_intersecting[not_zero] = intersect_ind

        laser_readings[rays_intersecting] = np.cross(wall_start - quad_pose, wall_vector)/ \
                                            ((denum[not_zero])[intersect_ind])

        # negative intersections
        negative_int = laser_readings < 0.0
        laser_readings[negative_int] = np.inf

        # put far readings to 100
        laser_readings[laser_readings > max_measurement] = 100

        return laser_readings

    def laser_readings_rectangle(self, rect_state, rect_dim):

        walls = np.zeros((4, 2, 2))
        rot_mat = np.array([[np.cos(rect_state[2]), -np.sin(rect_state[2])],
                            [np.sin(rect_state[2]), np.cos(rect_state[2])]])

        walls[0, 0] = rect_state[:2] + np.matmul(rot_mat, np.array([-rect_dim[0]/2.0, rect_dim[1]/2.0]))
        walls[0, 1] = rect_state[:2] + np.matmul(rot_mat, np.array([-rect_dim[0]/2.0, -rect_dim[1]/2.0]))
        walls[1, 0] = rect_state[:2] + np.matmul(rot_mat, np.array([-rect_dim[0] / 2.0, -rect_dim[1] / 2.0]))
        walls[1, 1] = rect_state[:2] + np.matmul(rot_mat, np.array([rect_dim[0] / 2.0, -rect_dim[1] / 2.0]))
        walls[2, 0] = rect_state[:2] + np.matmul(rot_mat, np.array([rect_dim[0] / 2.0, -rect_dim[1] / 2.0]))
        walls[2, 1] = rect_state[:2] + np.matmul(rot_mat, np.array([rect_dim[0] / 2.0, rect_dim[1] / 2.0]))
        walls[3, 0] = rect_state[:2] + np.matmul(rot_mat, np.array([rect_dim[0] / 2.0, rect_dim[1] / 2.0]))
        walls[3, 1] = rect_state[:2] + np.matmul(rot_mat, np.array([-rect_dim[0] / 2.0, rect_dim[1] / 2.0]))

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        lidar_readings = 100 * np.ones(num_samples)

        for it_walls in range(4):
            laser_readings_single_wall = self.laser_intersect_wall(walls[it_walls])
            lidar_readings[laser_readings_single_wall < lidar_readings] = laser_readings_single_wall[
                laser_readings_single_wall < lidar_readings]

        return lidar_readings

    def test_collision(self, state, radius):

        for it_obstacle in range(self.obstacle_num):
            if self.obstacle_type[it_obstacle] == 'cylinder':
                collision = self.cylinder_collision(state,
                                                    radius,
                                                    self.obstacle_pos[it_obstacle, :2],
                                                    self.obstacle_dim[it_obstacle])
            elif self.obstacle_type[it_obstacle] == 'rectangle':
                collision = self.rectangle_collision(state,
                                                     radius,
                                                     self.obstacle_pos[it_obstacle],
                                                     self.obstacle_dim[it_obstacle])
            elif self.obstacle_type[it_obstacle] == 'wall':
                collision = self.wall_collision(state,
                                                radius,
                                                self.obstacle_pos[it_obstacle, :2],
                                                self.obstacle_dim[it_obstacle])
            # elif self.obstacle_type[it_obstacle] == 'passage':
                # collision = self.passage_collision(state, radius, pasage_state, passage_dimension)
            else:
                print("UNKOWN OBSTACLE" + self.obstacle_type[it_obstacle])
                return True

            if collision:
                return collision

        return False

    def cylinder_collision(self, state, radius, cyl_state, cul_radius):
        if np.linalg.norm(state - cyl_state) < (cul_radius + radius):
            return True
        else:
            return False

    def rectangle_collision(self, state, radius, rect_state, rect_dimensions):
        vec_to_drone = state - rect_state[:2]
        rot_mat = np.array([[np.cos(rect_state[2]), np.sin(rect_state[2])],
                            [-np.sin(rect_state[2]), np.cos(rect_state[2])]])
        vec_to_drone = np.matmul(rot_mat, vec_to_drone)

        vec_to_drone_coll = np.zeros((5, 2))
        vec_to_drone_coll[0, :] = vec_to_drone + np.array([radius, 0.0])
        vec_to_drone_coll[1, :] = vec_to_drone + np.array([-radius, 0.0])
        vec_to_drone_coll[2, :] = vec_to_drone + np.array([0.0, radius])
        vec_to_drone_coll[3, :] = vec_to_drone + np.array([0.0, -radius])
        vec_to_drone_coll[4, :] = vec_to_drone - radius * vec_to_drone/np.linalg.norm(vec_to_drone)

        for it in range(5):
            point_checking = vec_to_drone_coll[it, :]
            check_1 = point_checking[0] >= -rect_dimensions[0] / 2.0
            check_2 = point_checking[0] <= rect_dimensions[0] / 2.0
            check_3 = point_checking[1] >= -rect_dimensions[1] / 2.0
            check_4 = point_checking[1] <= rect_dimensions[1] / 2.0
            if check_1 and check_2 and check_3 and check_4:
                return True

        return False

    def wall_collision(self, state, radius, wall_state, wall_end):

        vector_to_drone = state - wall_state
        vector_wall = wall_end - wall_state
        t_on_wall = np.dot(vector_to_drone, vector_wall) / np.dot(vector_wall, vector_wall)
        if t_on_wall > 1.0 or t_on_wall < 0.0:
            return False
        else:
            vector_ortogonal = state - (t_on_wall * vector_wall + wall_state)
            if np.linalg.norm(vector_ortogonal) < radius:
                return True
            else:
                return False
