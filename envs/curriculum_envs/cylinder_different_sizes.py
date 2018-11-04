import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import numpy.matlib
import time


class CylinderDiffSizesEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self):
        self.goal_pos = np.array([0.0, 0.0])
        self.start_pos = np.array([-10.0, 0.0, 0.0])
        self.bound_box = np.array([[-11.0, 2.0], [-4.0, 4.0]])

        self.state = self.start_pos
        self.viewer = None

        self.num_samples_laser = 35
        self.max_measurement_laser = 10.0
        self.laser_obs = 100.0 * np.ones(self.num_samples_laser)

        down_obs = np.zeros(self.num_samples_laser)
        up_obs = 100*np.ones(self.num_samples_laser)

        self.lidar_readings = np.ones(35)

        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(down_obs, up_obs)

        self.drone_radius = 0.4
        self.obstacle_num = 1
        self.obstacle_type = ['cylinder']

        # self.obstacle_radius = np.array([0.8])

        self.obstacle_radius = np.array([0.05])
        # self.obstacle_radius = np.array([0.2, 0.4, 0.8, 1.0])
        self.obstacle_pos = np.zeros((self.obstacle_num, 3))

        self.obstacle_pos = np.array([-5.5, 0.0, 0.0])
        # self.obstacle_pos = np.array([[-5.5, 0.5], [-7.0, 1.0], [-3.0, 1.5], [-4.5, -1.3]])

        self.lidar_range = 0
        self.action = 0

    def set_obstacle_radius(self, radius):
        self.obstacle_radius = radius

    def _seed(self, seed=None): # this is a hack, but no easy way to set parameters
        #self.set_obstacle_radius(seed['obstacle_radius'])
        #self.obstacle_num = seed['obstacle_num']
        #self.obstacle_pos = np.zeros((self.obstacle_num, 2))
        if seed['set_seed']:
            self.np_random, seed = seeding.np_random(seed['seed'])
        return [True]

    def _step(self, action):
        assert self.action_space.contains(action), "%r (%s) invalid" % (action, type(action))

        state = self.state
        old_state = self.state

        d_state = self.action_2_state_d(action)

        d_state = np.append(d_state, 0.0)

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

                collision = self.test_collision(self.state[:2], self.drone_radius)

                if collision:
                    reward = -1.0
                    done = True
                    break

            if not done:
                if self.state[0] > 0.0:
                    reward = 1.0
                    done = True
                else:
                    if np.linalg.norm(self.state[:2] - self.goal_pos) < np.linalg.norm(old_state[:2] - self.goal_pos):
                        reward = 0.01

                    # if action == 2:
                    #     reward = reward + 0.01



        observation = self.laser_readings()

        self.laser_obs = np.copy(observation)
        distance_vec = self.goal_pos - self.state[:2]

        # measurement = np.concatenate((observation, distance_vec))
        measurement = observation

        info = self.obstacle_pos


        return measurement, reward, done, info

    def action_2_state_d(self, action):

        action_divide_factor = 1

        return {
            0: np.divide([0.0, -0.5], action_divide_factor),
            1: np.divide([0.353, -0.353], action_divide_factor),
            2: np.divide([0.5, 0.0], action_divide_factor),
            3: np.divide([0.353, 0.353], action_divide_factor),
            4: np.divide([0.0, 0.5], action_divide_factor),

            # 0: np.array([0.0, -0.5]),
            # 1: np.array([0.353, -0.353]),
            # 2: np.array([0.5, 0.0]),
            # 3: np.array([0.353, 0.353]),
            # 4: np.array([0.0, 0.5]),

        }[action]



    def _reset(self):

        self.state = self.start_pos

        self.goal_pos[0] = 0.0
        self.goal_pos[1] = 0.0



        # obstacle pose
        self.obstacle_dim = []



        if self.obstacle_type[0] == 'cylinder':

            # self.obstacle_pos = np.array([-4.0, 0.0])
            # self.obstacle_pos = np.array([[-5.5, 0.5], [-7.0, 1.0], [-3.0, 1.5], [-4.5, -1.3]])

            # obstacle_x = -5
            # obstacle_y = 1

            obstacle_x = np.random.uniform(-7.0, -3.0, 1)
            obstacle_y = np.random.uniform(-1.0, 1.0, 1)

            self.obstacle_pos[0] = obstacle_x
            self.obstacle_pos[1] = obstacle_y

            radius = 0.5
            self.obstacle_dim.append(radius)

        elif self.obstacle_type[0] == 'rectangle':

            obstacle_x = np.random.uniform(-7.0, -3.0, 1)
            obstacle_y = np.random.uniform(-1.0, 1.0, 1)

            self.obstacle_pos[0] = obstacle_x
            self.obstacle_pos[1] = obstacle_y
            self.obstacle_pos[2] = np.pi

            dimensions = np.array([0.4, 0.6])
            self.obstacle_dim.append(dimensions)

        observation = self.laser_readings()
        self.laser_obs = np.copy(observation)
        distance_vec = self.goal_pos - self.state[0:2]

        #return np.concatenate((observation, distance_vec))
        return observation

    def _render(self, mode='human', close=False):
        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        screen_width = 1300
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

            goal = rendering.make_circle(screen_height / 6.0 * 0.5)
            goal.set_color(.0, 1.0, .0)
            self.goal_trans = rendering.Transform()
            goal.add_attr(self.goal_trans)
            self.viewer.add_geom(goal)

            obstacles = []
            self.obstacle_trans = []


            for it_obstacles in range(self.obstacle_num):
                if self.obstacle_type[0] == 'cylinder':

                    radius = screen_height / 8.0 * self.obstacle_radius[it_obstacles]
                    obstacles.append(rendering.make_circle(radius))
                    obstacles[it_obstacles].set_color(1.0, .0, .0)
                    self.obstacle_trans.append(rendering.Transform())
                    obstacles[it_obstacles].add_attr(self.obstacle_trans[it_obstacles])
                    self.viewer.add_geom(obstacles[it_obstacles])

                elif self.obstacle_type[0] == 'rectangle':

                    obst_dim = self.obstacle_dim[it_obstacles]
                    rect_width = obst_dim[0] * scale_width
                    rect_height = obst_dim[1] * scale_height
                    l, r, t, b = -rect_width / 2, rect_width / 2, rect_height / 2, -rect_height / 2
                    obstacles.append(rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)]))
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



        if self.state is None: return None

        drone_x = self.state[0] * scale_width + zero_width
        drone_y = self.state[1] * scale_height + zero_height
        self.drone_trans.set_translation(drone_x, drone_y)

        goal_x = self.goal_pos[0] * scale_width + zero_width
        goal_y = self.goal_pos[1] * scale_height + zero_height
        self.goal_trans.set_translation(goal_x, goal_y)

        if self.obstacle_num == 1:
            object_it_pos = self.obstacle_pos
            obstacle_x = object_it_pos[0] * scale_width + zero_width
            obstacle_y = object_it_pos[1] * scale_height + zero_height
            self.obstacle_trans[0].set_translation(obstacle_x, obstacle_y)
        else:
            for it_obstacles in range(self.obstacle_num):
                if self.obstacle_type[it_obstacles] == 'cylinder':
                    object_it_pos = self.objects_pos[it_obstacles, :]
                    obstacle_x = object_it_pos[0] * scale_width + zero_width
                    obstacle_y = object_it_pos[1] * scale_height + zero_height
                    self.obstacle_trans[it_obstacles].set_translation(obstacle_x, obstacle_y)

                elif self.obstacle_type[it_obstacles] == 'rectangle':
                    object_it_pos = self.objects_pos[it_obstacles]
                    obstacle_x = object_it_pos[0] * scale_width + zero_width
                    obstacle_y = object_it_pos[1] * scale_height + zero_height
                    self.obstacle_trans[it_obstacles].set_translation(obstacle_x, obstacle_y)











        if True:
            rays = np.linspace(-np.pi / 2, np.pi / 2, self.num_samples_laser)
            for it_laser in range(self.num_samples_laser):
                laser_reading_it = self.laser_obs[it_laser]
                # laser_reading_it = self.laser_readings()[it_laser]

                if laser_reading_it > self.max_measurement_laser:
                    laser_reading_it = self.max_measurement_laser



                laser_intersect = self.state[:2] + laser_reading_it * \
                                  np.array([np.cos(rays[it_laser]),
                                            np.sin(rays[it_laser])])



                laser_x = laser_intersect[0] * scale_width + zero_width
                laser_y = laser_intersect[1] * scale_height + zero_height
                self.laser_readings_array[it_laser].set_translation(laser_x, laser_y)



        return self.viewer.render(return_rgb_array=mode == 'rgb_array')




    def laser_readings(self):

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        lidar_readings = 2 * np.ones(num_samples)
        # random1 = np.random.random_integers(29)
        # random2 = np.random.random_integers(29)
        # random3 = np.random.random_integers(29)
        # random4 = np.random.random_integers(29)
        # random5 = np.random.random_integers(29)
        #
        # lidar_random = 0.5
        #
        # lidar_readings[random1] = lidar_random
        # lidar_readings[random2] = lidar_random
        # lidar_readings[random3] = lidar_random
        # lidar_readings[random4] = lidar_random
        # lidar_readings[random5] = lidar_random

        if self.obstacle_num == 0:
            if np.linalg.norm(self.state - self.obstacle_pos) <= max_measurement:
                single_obstacle_readings = self.laser_reading_single_cylinder(self.obstacle_pos, self.obstacle_radius)
                lidar_readings[single_obstacle_readings < lidar_readings] = single_obstacle_readings[
                    single_obstacle_readings < lidar_readings]
        else:
            for it in range(self.obstacle_num):
                if self.obstacle_type[it] == 'cylinder':
                    if np.linalg.norm(self.state - self.obstacle_pos) <= max_measurement:
                        single_obstacle_readings = \
                            self.laser_reading_single_cylinder(self.obstacle_pos, self.obstacle_radius[it])


                        lidar_readings[single_obstacle_readings < lidar_readings] = single_obstacle_readings[
                            single_obstacle_readings < lidar_readings]


                elif self.obstacle_type[it] == 'rectangle':
                    single_obstacle_readings = self.laser_readings_rectangle(self.obstacle_pos, self.obstacle_dim)
                    lidar_readings[single_obstacle_readings < lidar_readings] = single_obstacle_readings[
                        single_obstacle_readings < lidar_readings]


        return lidar_readings

    def laser_reading_single_cylinder(self, obstacle, radius_obs):

        circle_center = obstacle[:2]

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        rays = np.linspace(-np.pi / 2, np.pi / 2, num_samples)
        directions = np.array([np.cos(rays), np.sin(rays)])
        radius = radius_obs

        t0 = np.zeros(num_samples)

        quad_pose = self.state[:2]

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







    def laser_intersect_wall(self, wall_points):

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser


        laser_readings = np.inf*np.ones(num_samples)
        quad_pose = self.state[0:2]
        quad_angle = self.state[2]

        rays = np.linspace(quad_angle - np.pi / 2, quad_angle + np.pi / 2, num_samples)
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
        laser_readings[laser_readings > max_measurement] = 2

        return laser_readings


    def laser_readings_rectangle(self, rect_state, rect_dim):

        rect_dim = rect_dim[0]

        walls = np.zeros((4, 2, 2))
        rot_mat = np.array([[np.cos(rect_state[2]), -np.sin(rect_state[2])],
                            [np.sin(rect_state[2]), np.cos(rect_state[2])]])

        # walls[0, 0] = rect_state[:2]
        # walls[0, 1] = rect_state[:2]
        # walls[1, 0] = rect_state[:2]
        # walls[1, 1] = rect_state[:2]
        # walls[2, 0] = rect_state[:2]
        # walls[2, 1] = rect_state[:2]
        # walls[3, 0] = rect_state[:2]
        # walls[3, 1] = rect_state[:2]

        walls[0, 0] = rect_state[:2] + np.matmul(rot_mat, np.array([-rect_dim[0]/2.0, rect_dim[1]/2.0]))
        walls[0, 1] = rect_state[:2] + np.matmul(rot_mat, np.array([-rect_dim[0]/2.0, -rect_dim[1]/2.0]))
        walls[1, 0] = rect_state[:2] + np.matmul(rot_mat, np.array([-rect_dim[0]/2.0, -rect_dim[1]/2.0]))
        walls[1, 1] = rect_state[:2] + np.matmul(rot_mat, np.array([rect_dim[0]/2.0, -rect_dim[1]/2.0]))
        walls[2, 0] = rect_state[:2] + np.matmul(rot_mat, np.array([rect_dim[0]/2.0, -rect_dim[1]/2.0]))
        walls[2, 1] = rect_state[:2] + np.matmul(rot_mat, np.array([rect_dim[0]/2.0, rect_dim[1]/2.0]))
        walls[3, 0] = rect_state[:2] + np.matmul(rot_mat, np.array([rect_dim[0]/2.0, rect_dim[1]/2.0]))
        walls[3, 1] = rect_state[:2] + np.matmul(rot_mat, np.array([-rect_dim[0]/2.0, rect_dim[1]/2.0]))

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        lidar_readings = 2 * np.ones(num_samples)

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
                                                    self.obstacle_pos[:2],
                                                    self.obstacle_dim[it_obstacle])

            elif self.obstacle_type[it_obstacle] == 'rectangle':


                collision = self.rectangle_collision(state,
                                                     radius,
                                                     self.obstacle_pos,
                                                     self.obstacle_dim[it_obstacle])
            elif self.obstacle_type[it_obstacle] == 'wall':
                collision = self.wall_collision(state,
                                                radius,
                                                self.obstacle_pos,
                                                self.obstacle_dim[it_obstacle])
            # elif self.obstacle_type[it_obstacle] == 'passage':
                # collision = self.passage_collision(state, radius, pasage_state, passage_dimension)
            else:
                return True

            if collision:
                return collision

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

    def cylinder_collision(self, state, radius, cyl_state, cul_radius):
        if np.linalg.norm(state - cyl_state) < (cul_radius + radius):
            return True
        else:
            return False