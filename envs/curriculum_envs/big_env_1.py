import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import numpy.matlib


class BigEnv1(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self):

        self.maze_width = 20.0
        self.maze_height = 10.0
        self.maze_corners = np.array([[0.0, self.maze_height],
                                      [self.maze_width, self.maze_height],
                                      [self.maze_width, 0.0],
                                      [0.0, 0.0]])

        self.goal_pos = np.array([0.0, 0.0])
        self.start_pos = np.array([0.0, 0.0, 0.0])
        self.objects_pos = np.zeros((5, 2))
        self.bound_box = np.array([[-5.0, self.maze_width + 5.0], [-5.0, self.maze_height + 5.0]])
        self.goal_radius = 0.75
        self.drone_radius = 0.4

        self.state = np.copy(self.start_pos)
        self.viewer = None

        self.num_samples_laser = 80
        self.max_measurement_laser = 10.0
        self.laser_obs = 100.0*np.ones(self.num_samples_laser)

        down_obs = np.zeros(self.num_samples_laser)
        up_obs = 100 * np.ones(self.num_samples_laser)

        #self.action_space = spaces.Discrete(5 * 3)
        self.action_space = spaces.Discrete(7)
        self.observation_space = spaces.Box(down_obs, up_obs)
        self.manual_pose = False

        # self.walls = np.zeros((16, 2, 2))

        self.walls = np.zeros((8, 2, 2))

        # walls positions

        vert_up_walls_num = 5
        for it_walls_vert_up in range(vert_up_walls_num):
            wall = np.array([[0.0, 0.0], [0.0, 4.0]])
            x_offset = (it_walls_vert_up + 1) * 3.0
            y_offset = 6.0
            wall += np.array([x_offset, y_offset])
            self.walls[it_walls_vert_up] = wall

        count_offset = vert_up_walls_num
        # hor_up_walls_num = 5
        hor_up_walls_num = 0
        for it_walls_hor_up in range(hor_up_walls_num):
            wall = np.array([[0.0, 0.0], [2.0, 0.0]])
            x_offset = it_walls_hor_up * 3.0
            y_offset = 6.0
            wall += np.array([x_offset, y_offset])
            self.walls[count_offset + it_walls_hor_up] = wall

        count_offset += hor_up_walls_num
        vert_down_walls_num = 2
        for it_walls_vert_down in range(vert_down_walls_num):
            wall = np.array([[0.0, 0.0], [0.0, 4.0]])
            x_offset = (it_walls_vert_down + 1) * 6.0
            y_offset = 0.0
            wall += np.array([x_offset, y_offset])
            self.walls[count_offset + it_walls_vert_down] = wall

        count_offset += vert_down_walls_num
        # hor_down_walls_num = 2
        hor_down_walls_num = 0
        for it_walls_hor_down in range(hor_down_walls_num):
            wall = np.array([[0.0, 0.0], [5.0, 0.0]])
            x_offset = it_walls_hor_down * 6.0
            y_offset = 4.0
            wall += np.array([x_offset, y_offset])
            self.walls[count_offset + it_walls_hor_down] = wall

        # small room
        count_offset += hor_down_walls_num
        wall = np.array([[self.maze_width - 2.0, self.maze_height - 3], [self.maze_width - 2.0, self.maze_height]])
        self.walls[count_offset + 0] = wall
        # wall = np.array([[self.maze_width - 2.0, self.maze_height - 4.0], [self.maze_width, self.maze_height- 4.0]])
        # self.walls[count_offset + 1] = wall

    def _seed(self, seed=None):
        if seed['set_seed']:
            self.np_random, seed = seeding.np_random(seed['seed'])
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
            reward = 0.0
        else:

            collision_walls = self.test_collision(self.state[0:2], self.drone_radius)

            if collision_walls:
                reward = -1.0
                done = True
            else:
                if np.linalg.norm(self.goal_pos - self.state[0:2]) < 0.75:
                    reward = 1.0
                    done = True
                else:
                    if np.linalg.norm(self.state[0:2] - self.goal_pos) < np.linalg.norm(old_state[0:2] - self.goal_pos):
                        #reward = 0.0005
                        #reward = 0.01
                        reward = 0.001

        observation = self.laser_readings()
        self.laser_obs = np.copy(observation)
        distance_vec = self.goal_pos - self.state[0:2]
        distance = np.linalg.norm(distance_vec)
        angle = np.arctan2(distance_vec[1], distance_vec[0])
        angle = angle - self.state[2]
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        goal_measurements = np.array([distance, angle])

        measurement = np.concatenate((observation, goal_measurements))
        # measurement = goal_measurements

        info = self.objects_pos

        return measurement, reward, done, info

    # def action_2_state_d(self, action):
    #     return {
    #         0: np.array([0.2, 0.0, 0.0]),
    #         1: np.array([0.1414, 0.1414, 0.0]),
    #         2: np.array([0.0, 0.2, 0.0]),
    #         3: np.array([0.0, -0.2, 0.0]),
    #         4: np.array([0.1414, -0.1414, 0.0]),
    #         5: np.array([0.2, 0.0, np.pi/8]),
    #         6: np.array([0.1414, 0.1414, np.pi/8]),
    #         7: np.array([0.0, 0.2, np.pi/8]),
    #         8: np.array([0.0, -0.2, np.pi/8]),
    #         9: np.array([0.1414, -0.1414, np.pi/8]),
    #         10: np.array([0.2, 0.0, -np.pi / 8]),
    #         11: np.array([0.1414, 0.1414, -np.pi / 8]),
    #         12: np.array([0.0, 0.2, -np.pi / 8]),
    #         13: np.array([0.0, -0.2, -np.pi / 8]),
    #         14: np.array([0.1414, -0.1414, -np.pi / 8]),
    #     }[action]

    # def action_2_state_d(self, action):
    #     return {
    #         0: np.array([0.5, 0.0, 0.0]),
    #         1: np.array([0.353, 0.353, 0.0]),
    #         2: np.array([0.0, 0.5, 0.0]),
    #         3: np.array([0.0, -0.5, 0.0]),
    #         4: np.array([0.353, -0.353, 0.0]),
    #         5: np.array([0.5, 0.0, np.pi/8]),
    #         6: np.array([0.353, 0.353, np.pi/8]),
    #         7: np.array([0.0, 0.5, np.pi/8]),
    #         8: np.array([0.0, -0.5, np.pi/8]),
    #         9: np.array([0.353, -0.353, np.pi/8]),
    #         10: np.array([0.5, 0.0, -np.pi / 8]),
    #         11: np.array([0.353, 0.353, -np.pi / 8]),
    #         12: np.array([0.0, 0.5, -np.pi / 8]),
    #         13: np.array([0.0, -0.5, -np.pi / 8]),
    #         14: np.array([0.353, -0.353, -np.pi / 8]),
    #     }[action]

    def action_2_state_d(self, action):
        return {
            0: np.array([0.5, 0.0, 0.0]),
            1: np.array([0.353, 0.353, 0.0]),
            2: np.array([0.0, 0.5, 0.0]),
            3: np.array([0.0, -0.5, 0.0]),
            4: np.array([0.353, -0.353, 0.0]),
            5: np.array([0.0, 0.0, np.pi/8]),
            6: np.array([0.0, 0.0, -np.pi/8]),
        }[action]

    def _reset(self):

        self.state = np.copy(self.start_pos)

        # goal pose
        collision = True
        while collision:
            self.goal_pos[0] = np.random.uniform(0.0, self.maze_width, 1)
            self.goal_pos[1] = np.random.uniform(0.0, self.maze_height, 1)
            # self.goal_pos = np.array([17.0, 2.0])
            collision = self.test_collision(self.goal_pos, self.goal_radius)

        # start_pose
        collision = True
        while collision:
            self.state[0] = np.random.uniform(0.0, self.maze_width, 1)
            self.state[1] = np.random.uniform(0.0, self.maze_height, 1)
            self.state[2] = np.random.uniform(-np.pi, np.pi, 1)
            # self.state = np.array([17.0, 9.0, -np.pi/2.0])
            # self.state = np.array([1.0, 5.0, 0.0])
            collision = self.test_collision(self.state[0:2], self.drone_radius)

        # observation reading
        observation = self.laser_readings()
        self.laser_obs = np.copy(observation)
        distance_vec = self.goal_pos - self.state[0:2]
        distance = np.linalg.norm(distance_vec)
        angle = np.arctan2(distance_vec[1], distance_vec[0])
        angle = angle - self.state[2]
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        goal_measurements = np.array([distance, angle])

        return np.concatenate((observation, goal_measurements))
        # return goal_measurements

    def test_collision(self, position, radius):

        for it_maze in range(np.size(self.maze_corners, 0)):
            wall_points = self.get_wall(it_maze)
            wall_coll = self.wall_collision(wall_points, position, radius)
            if wall_coll:
                return True

        for it_walls in range(np.shape(self.walls)[0]):
            wall_coll = self.wall_collision(self.walls[it_walls], position, radius)
            if wall_coll:
                return True

        return False

    def wall_collision(self, wall_points, drone_pose, radius):

        vector_to_drone = drone_pose - wall_points[0, :]
        vector_wall = wall_points[1, :] - wall_points[0, :]
        t_on_wall = np.dot(vector_to_drone, vector_wall)/np.dot(vector_wall, vector_wall)
        if t_on_wall > 1.0 or t_on_wall < 0.0:
            return False
        else:
            vector_ortogonal = drone_pose - (t_on_wall*vector_wall + wall_points[0, :])
            if np.linalg.norm(vector_ortogonal) < radius:
                return True
            else:
                return False

    def get_wall(self, it_walls):

        if it_walls < (np.size(self.maze_corners, 0) - 1):
            wall_points = self.maze_corners[it_walls:it_walls + 2, :]
        else:
            wall_points = np.array([self.maze_corners[it_walls], self.maze_corners[0]])

        return wall_points

    def laser_readings(self):

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        lidar_readings = 100*np.ones(num_samples)

        for it_maze in range(np.size(self.maze_corners, 0)):
            wall_points = self.get_wall(it_maze)
            single_obstacle_readings = self.laser_intersect_wall(wall_points, self.state)
            lidar_readings[single_obstacle_readings < lidar_readings] = single_obstacle_readings[
                single_obstacle_readings < lidar_readings]

        for it_walls in range(np.shape(self.walls)[0]):
            single_obstacle_readings = self.laser_intersect_wall(self.walls[it_walls], self.state)
            lidar_readings[single_obstacle_readings < lidar_readings] = single_obstacle_readings[
                single_obstacle_readings < lidar_readings]

        return lidar_readings

    def _render(self, mode='human', close=False):
        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        screen_width = 1000
        screen_height = int(screen_width * (self.bound_box[1, 1] - self.bound_box[1, 0]) /\
                                           (self.bound_box[0, 1] - self.bound_box[0, 0]))

        scale_width = screen_width / (self.bound_box[0, 1] - self.bound_box[0, 0])
        scale_height = screen_height / (self.bound_box[1, 1] - self.bound_box[1, 0])

        zero_width = scale_width * (-self.bound_box[0, 0])
        zero_height = scale_height * (-self.bound_box[1, 0])

        drone_width = 20
        drone_height = 20

        wall_height = 5

        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(screen_width, screen_height)

            l, r, t, b = -drone_width / 2, drone_width / 2, drone_height / 2, -drone_height / 2
            drone = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            self.drone_trans = rendering.Transform()
            drone.add_attr(self.drone_trans)
            self.viewer.add_geom(drone)

            goal = rendering.make_circle(scale_height * self.goal_radius)
            goal.set_color(.0, 1.0, .0)
            self.goal_trans = rendering.Transform()
            goal.add_attr(self.goal_trans)
            self.viewer.add_geom(goal)

            walls_maze = []
            self.walls_trans_maze = []
            for it_walls in range(np.size(self.maze_corners, 0)):
                wall_points = self.get_wall(it_walls)

                wall_vector = wall_points[1, :] - wall_points[0, :]
                wall_width = np.linalg.norm(wall_vector) * scale_width #screen_width/18.0
                l, r, t, b = 0, wall_width, 0, wall_height
                walls_maze.append(rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)]))
                walls_maze[it_walls].set_color(1.0, 0.0, 0.0)
                self.walls_trans_maze.append(rendering.Transform())
                walls_maze[it_walls].add_attr(self.walls_trans_maze[it_walls])
                self.viewer.add_geom(walls_maze[it_walls])

            walls_array = []
            self.walls_trans_array = []
            for it_walls in range(np.size(self.walls, 0)):
                wall_points = self.walls[it_walls]

                wall_vector = wall_points[1, :] - wall_points[0, :]
                wall_width = np.linalg.norm(wall_vector) * scale_width #screen_width/18.0
                l, r, t, b = 0, wall_width, 0, wall_height
                walls_array.append(rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)]))
                walls_array[it_walls].set_color(1.0, 0.0, 0.0)
                self.walls_trans_array.append(rendering.Transform())
                walls_array[it_walls].add_attr(self.walls_trans_array[it_walls])
                self.viewer.add_geom(walls_array[it_walls])

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

        for it_walls in range(np.size(self.maze_corners, 0)):
            wall_pose = self.maze_corners[it_walls]
            wall_x = wall_pose[0] * scale_width + zero_width
            wall_y = wall_pose[1] * scale_height + zero_height
            self.walls_trans_maze[it_walls].set_translation(wall_x, wall_y)

            wall_points = self.get_wall(it_walls)
            wall_vector = wall_points[1, :] - wall_points[0, :]
            rotation_angle = np.arctan2(wall_vector[1], wall_vector[0])
            self.walls_trans_maze[it_walls].set_rotation(rotation_angle)

        for it_walls in range(np.size(self.walls, 0)):
            wall_points = self.walls[it_walls]
            wall_pose = wall_points[0, :]
            wall_x = wall_pose[0] * scale_width + zero_width
            wall_y = wall_pose[1] * scale_height + zero_height
            self.walls_trans_array[it_walls].set_translation(wall_x, wall_y)

            wall_vector = wall_points[1, :] - wall_points[0, :]
            rotation_angle = np.arctan2(wall_vector[1], wall_vector[0])
            self.walls_trans_array[it_walls].set_rotation(rotation_angle)

        if True:
            rays = np.linspace(-np.pi / 2, np.pi / 2, self.num_samples_laser)
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

    def laser_reading_single_cylinder(self, obstacle):

        circle_center = obstacle

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        rays = np.linspace(-np.pi / 2, np.pi / 2, num_samples)
        directions = np.array([np.cos(rays), np.sin(rays)])
        radius = 0.2

        t0 = np.zeros(num_samples)

        quad_pose = self.state

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

        intersection_distace = t0
        intersection_distace[t0 > max_measurement] = 100

        return intersection_distace

    def laser_intersect_wall(self, wall_points, drone_pose):

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        laser_readings = np.inf*np.ones(num_samples)
        quad_pose = drone_pose[0:2]
        quad_angle = drone_pose[2]

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
        laser_readings[laser_readings > max_measurement] = 100

        return laser_readings