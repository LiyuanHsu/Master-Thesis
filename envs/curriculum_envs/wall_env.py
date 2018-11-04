import gym
from gym import spaces
from gym.utils import seeding
import numpy as np
import numpy.matlib


class WallEnv(gym.Env):
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }

    def __init__(self):
        self.goal_pos = np.array([0.0, 0.0])
        self.start_pos = np.array([-14.0, 0.0])
        self.objects_pos = np.array([-5.0, 0.0])
        self.bound_box = np.array([[-15.0, 2.0], [-3.0, 3.0]])

        self.state = self.start_pos
        self.viewer = None

        self.num_samples_laser = 80
        self.max_measurement_laser = 10.0

        down_obs = np.zeros(self.num_samples_laser)
        up_obs = 100*np.ones(self.num_samples_laser)

        self.action_space = spaces.Discrete(8)
        self.observation_space = spaces.Box(down_obs, up_obs)

        self.obstacle_num = 1
        self.obstacle_height = 2.0
        self.obstacle_width = 0.5
        self.obstacle_orientation = 0.0

        self.x_obst_rand = [-8.0, -3.0]
        self.y_obst_rand = [-3.0, 1.0]

        self.maze_corners = np.array([])

    def _seed(self, seed=None): # this is a hack, but no easy way to set parameters
        self.obstacle_orientation = seed['obstacle_orientation']
        self.x_obst_rand = seed['x_obst_rand']
        self.y_obst_rand = seed['y_obst_rand']
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
            reward = -1.0  # TODO: maybe not to penalize for going out
        else:
            wall_collsion = self.maze_collision(self.state)
            if wall_collsion:
                reward = -1.0
                done = True
            else:
                if self.state[0] > 0.0:
                    reward = 1.0
                    done = True
                else:
                    if np.linalg.norm(self.state - self.goal_pos) < np.linalg.norm(old_state - self.goal_pos):
                        reward = 0.01
                    else:
                        reward = -0.01  # TODO: think about this cost

                # if np.linalg.norm(self.goal_pos - self.state) < 0.75:
                #    reward = 1.0
                #    done = True
                # else:
                #    reward = 0.01*(1.0 - (np.linalg.norm(self.goal_pos - self.state)/np.abs(self.bound_box[0, 0]))**2)

        observation = self.laser_intersects_maze(self.state)
        distance_vec = self.goal_pos - self.state

        measurement = np.concatenate((observation, distance_vec))
        #measurement = observation

        info = self.objects_pos

        return measurement, reward, done, info

    # def action_2_state_d(self, action):
    #     return {
    #         0: np.array([0.2, 0.0]),
    #         1: np.array([0.1414, 0.1414]),
    #         2: np.array([0.0, 0.2]),
    #         3: np.array([-0.1414, 0.1414]),
    #         4: np.array([-0.2, 0.0]),
    #         5: np.array([-0.1414, -0.1414]),
    #         6: np.array([0.0, -0.2]),
    #         7: np.array([0.1414, -0.1414]),
    #     }[action]

    def action_2_state_d(self, action):
        return {
            0: np.array([0.5, 0.0]),
            1: np.array([0.353, 0.353]),
            2: np.array([0.0, 0.5]),
            3: np.array([-0.353, 0.353]),
            4: np.array([-0.5, 0.0]),
            5: np.array([-0.353, -0.353]),
            6: np.array([0.0, -0.5]),
            7: np.array([0.353, -0.353]),
        }[action]

    def _reset(self):

        self.state = self.start_pos

        self.objects_pos[0] = np.random.uniform(self.x_obst_rand[0], self.x_obst_rand[1], self.obstacle_num)
        self.objects_pos[1] = np.random.uniform(self.y_obst_rand[0], self.y_obst_rand[1], self.obstacle_num)

        point_1_x = self.objects_pos[0] + self.obstacle_height * np.sin(self.obstacle_orientation)
        point_1_y = self.objects_pos[1] + self.obstacle_height * np.cos(self.obstacle_orientation)

        point_2_x = point_1_x + self.obstacle_width * np.sin(self.obstacle_orientation + np.pi/2)
        point_2_y = point_1_y + self.obstacle_width * np.cos(self.obstacle_orientation + np.pi/2)

        point_3_x = point_2_x + self.obstacle_height * np.sin(self.obstacle_orientation + np.pi)
        point_3_y = point_2_y + self.obstacle_height * np.cos(self.obstacle_orientation + np.pi)

        self.maze_corners = np.array([[self.objects_pos[0], self.objects_pos[1]],
                                      [point_1_x, point_1_y],
                                      [point_2_x, point_2_y],
                                      [point_3_x, point_3_y]])

        # self.goal_pos[0] = np.random.uniform(-2.0, 1.5, self.obstacle_num)
        # self.goal_pos[1] = np.random.uniform(-2.5, 2.5, self.obstacle_num)
        self.goal_pos[0] = 0.0
        self.goal_pos[1] = 0.0

        observation = self.laser_intersects_maze(self.state)
        distance_vec = self.goal_pos - self.state

        return np.concatenate((observation, distance_vec))
        #return observation

    def _render(self, mode='human', close=False):
        if close:
            if self.viewer is not None:
                self.viewer.close()
                self.viewer = None
            return

        screen_width = 1000
        screen_height = int(screen_width * (self.bound_box[1, 1] - self.bound_box[1, 0]) / \
                            (self.bound_box[0, 1] - self.bound_box[0, 0]))

        scale_width = screen_width / (self.bound_box[0, 1] - self.bound_box[0, 0])
        scale_height = screen_height / (self.bound_box[1, 1] - self.bound_box[1, 0])

        zero_width = scale_width * (-self.bound_box[0, 0])
        zero_height = scale_height * (-self.bound_box[1, 0])

        drone_width = 20
        drone_height = 20

        wall_height = 20

        if self.viewer is None:
            from gym.envs.classic_control import rendering
            self.viewer = rendering.Viewer(screen_width, screen_height)

            l, r, t, b = -drone_width / 2, drone_width / 2, drone_height / 2, -drone_height / 2
            drone = rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)])
            self.drone_trans = rendering.Transform()
            drone.add_attr(self.drone_trans)
            self.viewer.add_geom(drone)

            walls = []
            self.walls_trans = []
            for it_walls in range(np.size(self.maze_corners, 0)):
                wall_points = self.get_wall(it_walls)

                wall_vector = wall_points[1, :] - wall_points[0, :]
                wall_width = np.linalg.norm(wall_vector) * scale_width  # screen_width/18.0
                l, r, t, b = 0, wall_width, 0, wall_height
                walls.append(rendering.FilledPolygon([(l, b), (l, t), (r, t), (r, b)]))
                walls[it_walls].set_color(1.0, 0.0, 0.0)
                self.walls_trans.append(rendering.Transform())
                walls[it_walls].add_attr(self.walls_trans[it_walls])
                self.viewer.add_geom(walls[it_walls])

        if self.state is None: return None

        drone_x = self.state[0] * scale_width + zero_width
        drone_y = self.state[1] * scale_height + zero_height
        self.drone_trans.set_translation(drone_x, drone_y)

        for it_walls in range(np.size(self.maze_corners, 0)):
            wall_pose = self.maze_corners[it_walls]
            wall_x = wall_pose[0] * scale_width + zero_width
            wall_y = wall_pose[1] * scale_height + zero_height
            self.walls_trans[it_walls].set_translation(wall_x, wall_y)

            wall_points = self.get_wall(it_walls)
            wall_vector = wall_points[1, :] - wall_points[0, :]
            rotation_angle = np.arctan2(wall_vector[1], wall_vector[0])
            self.walls_trans[it_walls].set_rotation(rotation_angle)

        return self.viewer.render(return_rgb_array=mode == 'rgb_array')

    def laser_readings(self):

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        lidar_readings = 100*np.ones(num_samples)

        for it in range(self.obstacle_num):
            if np.linalg.norm(self.state - self.objects_pos[it, :]) <= max_measurement:

                single_obstacle_readings = self.laser_reading_single_cylinder(self.objects_pos[it, :])
                lidar_readings[single_obstacle_readings < lidar_readings] = single_obstacle_readings[single_obstacle_readings < lidar_readings]

        return lidar_readings

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

        rays = np.linspace(-np.pi / 2, np.pi / 2, num_samples)
        directions = np.transpose(np.array([np.cos(rays), np.sin(rays)]))

        wall_vector = wall_points[1, :] - wall_points[0, :]
        wall_start = wall_points[0, :]

        denum = np.cross(directions, wall_vector)

        # determine if lines are parallel
        not_zero = np.logical_not(np.equal(denum, 0.0))

        wall_intersection = np.cross(wall_start - drone_pose, directions[not_zero, :])/denum[not_zero]

        # find intersections in range 0 to 1
        wall_intersection_less_zero = wall_intersection > 0.0
        wall_intersection_grater_zero = wall_intersection < 1.0
        intersect_ind = np.logical_and(wall_intersection_less_zero, wall_intersection_grater_zero)

        # rays that intersect
        rays_intersecting = False*np.ones(num_samples, dtype=bool)
        rays_intersecting[not_zero] = intersect_ind

        laser_readings[rays_intersecting] = np.cross(wall_start - drone_pose, wall_vector)/ \
                                            ((denum[not_zero])[intersect_ind])

        # negative intersections
        negative_int = laser_readings < 0.0
        laser_readings[negative_int] = np.inf

        # put far readings to 100
        laser_readings[laser_readings > max_measurement] = 100

        return laser_readings

    def laser_intersects_maze(self, drone_pose):

        num_samples = self.num_samples_laser
        max_measurement = self.max_measurement_laser

        lidar_readings = 100 * np.ones(num_samples)

        for it_walls in range(np.size(self.maze_corners, 0)):
            wall_points = self.get_wall(it_walls)

            single_obstacle_readings = self.laser_intersect_wall(wall_points, drone_pose)
            lidar_readings[single_obstacle_readings < lidar_readings] = single_obstacle_readings[
                single_obstacle_readings < lidar_readings]

        return lidar_readings

    def wall_collision(self, wall_points, drone_pose):

        vector_to_drone = drone_pose - wall_points[0, :]
        vector_wall = wall_points[1, :] - wall_points[0, :]
        t_on_wall = np.dot(vector_to_drone, vector_wall)/np.dot(vector_wall, vector_wall)
        if t_on_wall > 1.0 or t_on_wall < 0.0:
            return False
        else:
            vector_ortogonal = drone_pose - (t_on_wall*vector_wall + wall_points[0, :])
            if np.linalg.norm(vector_ortogonal) < 0.40:
                return True
            else:
                return False

    def maze_collision(self, drone_pose):

        for it_walls in range(np.size(self.maze_corners, 0)):

            wall_points = self.get_wall(it_walls)

            detct_collision = self.wall_collision(wall_points, drone_pose)

            if detct_collision:
                return True

        return False

    def get_wall(self, it_walls):

        if it_walls < (np.size(self.maze_corners, 0) - 1):
            wall_points = self.maze_corners[it_walls:it_walls + 2, :]
        else:
            wall_points = np.array([self.maze_corners[it_walls], self.maze_corners[0]])

        return wall_points
