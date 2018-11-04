import numpy as np
from gym import utils
from gym.envs.mujoco import mujoco_env
import scipy.interpolate as si
from gym.utils import seeding

class DroneEnv(mujoco_env.MujocoEnv, utils.EzPickle):
    def __init__(self):
        self.spline_degree = 3
        self.prediction_len = 4
        self.action_sequence = np.zeros(self.prediction_len + self.prediction_len + self.prediction_len, dtype=np.int)
        self.spline_func, self.quad_path, self.spl_projections, self.max_t = \
            DroneEnv.get_b_spline(self.action_sequence, self.spline_degree)
        self.spline_der = self.spline_func.derivative()
        self.t_param = 0.0
        self.t_old = 0.0
        self.ref_ind_cur = 0

        self.pos = np.array([0.0, 0.0, 1.0])

        self.down_state_limit = np.array([-5.0, -5.0, 0.2, -10.0, -10.0, -10.0, -np.pi / 2, -np.pi / 2])
        self.up_state_limit = np.array([10.0, 5.0, 6.0, 10.0, 10.0, 10.0, np.pi / 2, np.pi / 2])
        self.state = np.zeros(8)
        self.state[:3] = self.pos

        self.ref_pose = np.array([0.0, 0.0, 1.0])
        self.ref_actions = np.zeros(self.prediction_len, dtype=np.int32)

        self.action_size = 4

        mujoco_env.MujocoEnv.__init__(self, '/home/stevsics/AIT_code/gym/gym/envs/drone_mujoco/assets/drone2.xml', 5)
        #mujoco_env.MujocoEnv.__init__(self, 'hopper.xml', 5)
        utils.EzPickle.__init__(self)

    def _seed(self, seed=None):
        if not seed == None:
            self.prediction_len = seed['prediction_len']
            action_sequence = seed['action_sequence']
            self.action_sequence = np.concatenate(
                (np.zeros(self.prediction_len), action_sequence, np.zeros(self.prediction_len)))
            self.spline_func, self.quad_path, self.spl_projections, self.max_t = \
                DroneEnv.get_b_spline(self.action_sequence, self.spline_degree)
            self.spline_der = self.spline_func.derivative()
            self.t_param = 0.0
            self.t_old = 0.0
            self.ref_ind_cur = 0
            self.ref_actions = self.get_ref_actions()
            observation = self._get_obs()

            self.np_random, seed = seeding.np_random(None) # TODO: if you want to seed add option

            return_value = {'observation': observation}
        else:
            self.np_random, seed = seeding.np_random(None)
            return_value = [None]

        return return_value

    def _step(self, action):
        # execute sim

        # action = np.array([0.5, 0.6, 0.5, 0.6])
        if not np.shape(action)[0] == self.action_size:
            action = np.zeros(self.action_size)
        action = np.clip(action, 0.0, 1.0)
        # 459023 constant to scale input to omega^2 (omega - propeler rotation speed)
        rottor_speed_squared = 459023.0 * action

        k_F = 8.54858e-6
        k_M = 0.016 * k_F
        L = 0.1
        l_xy = 0.0707
        mat_v2u = np.array([[k_F, k_F, k_F, k_F],
                            [k_F * l_xy, k_F * l_xy, -k_F * l_xy, -k_F * l_xy],
                            [-k_F * l_xy, k_F * l_xy, k_F * l_xy, -k_F * l_xy],
                            [k_M, -k_M, k_M, -k_M]])

        u = np.matmul(mat_v2u, rottor_speed_squared)

        pos_vec = self.model.data.qpos.ravel().copy()
        quat_drone = pos_vec[3:]

        # vec_z = np.array([0.0, 0.0, 1.0]) as quat
        q_z = np.array([0.0, 0.0, 0.0, u[0]])
        vec_control = self.q_mult(self.q_mult(quat_drone, q_z), self.q_conjugate(quat_drone))[1:]

        action = np.array([vec_control[0], vec_control[1], vec_control[2], u[1], u[2], u[3]])
        #action = np.array([0.5, 0.0, 7.1, 0.0, 0.0, 0.01])
        #action = np.array([0.0, 0.0, 7.1, 0.0, 0.05, 0.01])

        self.do_simulation(action, self.frame_skip)

        # calculating closest point on spline
        pos_vec = self.model.data.qpos.ravel().copy()
        self.pos = pos_vec[:3]

        t_closest, distance = self.get_closest_t()

        vel_t = (t_closest - self.t_old) / self.dt
        self.t_old = t_closest

        der_at_t = self.spline_der(t_closest)
        der_at_t = der_at_t / np.linalg.norm(der_at_t)

        self.ref_actions = self.get_ref_actions()

        # get observations
        observation = self._get_obs()

        # calculate reward
        vel_vec = self.model.data.qvel.ravel().copy()
        vel_quad = vel_vec[:3]
        w1, x1, y1, z1 = pos_vec[3:]
        R1 = w1 ** 2 + x1 ** 2 - y1 ** 2 - z1 ** 2
        R2 = 2*(x1 * y1 + w1 * z1)
        yaw = np.arctan2(R2, R1)

        # survive_cost = 0.0
        # vel_reward = - 0.1 * (1.5 - np.dot(der_at_t[0:2], vel_quad[:2])) ** 2
        # ec_reward = - 5 * distance ** 2
        # yaw_reward = - 5 * np.abs(0.0 - yaw) ** 2
        # ctrl_cost = -0.01 * 2.0 * np.square(action).sum()

        survive_cost = 0.0
        vel_reward = - 0.1 * 0.1 * (1.5 - np.dot(der_at_t[0:2], vel_quad[:2])) ** 2
        ec_reward = - 0.1 * 5 * distance ** 2
        yaw_reward = - 0.1 * 5 * np.abs(0.0 - yaw) ** 2
        ctrl_cost = - 0.1 * 0.01 * 2.0 * np.square(action).sum()

        reward = vel_reward + survive_cost + ctrl_cost + ec_reward + yaw_reward

        # TODO: cost does not penalize if drone goes out of range because its very far from line, range also could be smaller
        # TODO: we could penalize if control signal out of bounds, but for now only ctrl_cost as in mujoco envs

        # check if outside of the box so we can interrupt training

        self.state[:3] = self.pos
        self.state[3:6] = vel_vec[:3]
        self.state[6:] = np.zeros(2)  # TODO: stop if drone flips if does not work

        # drone z axis

        quat_drone = pos_vec[3:]
        z_axis = np.array([0.0, 0.0, 0.0, 1.0])
        z_drone = self.q_mult(self.q_mult(quat_drone, z_axis), self.q_conjugate(quat_drone))[1:]

        done = False
        tube_radius = 0.75
        if distance > 0.75:
            done = True
        elif self.ref_ind_cur >= (np.shape(self.action_sequence)[0] - 1):
            done = True
        else:
            for it_states in range(np.shape(self.up_state_limit)[0]):
                if self.state[it_states] > self.up_state_limit[it_states]:
                    done_it = True
                elif self.state[it_states] < self.down_state_limit[it_states]:
                    done_it = True
                else:
                    done_it = False

                if done_it:
                    done = True
                    break

            if not done:
                if z_drone[2] <= 0:
                    done = True

        if done:
            if self.state[2] < self.down_state_limit[2]:
                reward += -1000.0
            elif z_drone[2] <= 0:
                reward += -1000.0
            elif distance > 0.75:
                reward += -1000.0

        info = {'state': self.state}
        info['t_min'] = t_closest

        return observation, reward, done, info

    def _get_obs(self):
        return np.concatenate([
            self.ref_actions.astype(dtype=np.float32),
            self.model.data.qpos.flat[2:],
            self.model.data.qvel.flat
        ])

    def reset_model(self):
        qpos = self.init_qpos  # + self.np_random.uniform(low=-.1, high=.1, size=self.model.nq)
        # qpos[3:] = np.array([0.9238795325112867, 0.0, 0.3826834323650898, 0.0])
        qvel = self.init_qvel  # + self.np_random.randn(self.model.nv) * .1
        self.set_state(qpos, qvel)

        self.t_param = 0.0
        self.t_old = 0.0
        self.ref_ind_cur = 0
        self.ref_pose = np.array([0.0, 0.0, 1.0])
        self.state = np.zeros(8)
        self.pos = qpos[:3]
        self.state[:3] = qpos[:3]
        self.state[3:6] = qvel[:3]
        self.ref_actions = self.get_ref_actions()
        observation = self._get_obs()

        return observation

    def viewer_setup(self):
        self.viewer.cam.distance = self.model.stat.extent * 0.5

    def q_mult(self, q1, q2):
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
        z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
        return np.array([w, x, y, z])

    def q_conjugate(self, q):
        w, x, y, z = q
        return (w, -x, -y, -z)

    def get_closest_t(self):

        search_range = np.linspace(self.t_param - 1.0, self.t_param + 1.0,
                                   1000)  # TODO: search parameter can slow down things
        search_range = np.clip(search_range, 0.0, self.max_t)

        t_poses = self.spline_func(search_range)
        distances = np.linalg.norm(t_poses - self.pos, axis=1)
        min_indx = np.argmin(distances)

        self.t_param = search_range[min_indx]
        return self.t_param, distances[min_indx]

    def get_ref_actions(self):

        discrete_tangent_vec = self.spl_projections[self.ref_ind_cur + 1] - self.spl_projections[self.ref_ind_cur]
        quad_vec = self.pos - self.spl_projections[self.ref_ind_cur]
        quad_vec_proj = np.dot(quad_vec, discrete_tangent_vec) / np.linalg.norm(discrete_tangent_vec)
        if quad_vec_proj > np.linalg.norm(discrete_tangent_vec):  # TODO: RL might go back, should we encount this ?
            self.ref_ind_cur += 1
            self.ref_pose = self.pos
        elif quad_vec_proj < 0.0:
            self.ref_ind_cur -= 1

        if self.ref_ind_cur < 0:
            self.ref_ind_cur = 0

        if (self.ref_ind_cur + self.prediction_len) < self.action_sequence.shape[0]:
            ref_actions = self.action_sequence[self.ref_ind_cur: self.ref_ind_cur + self.prediction_len]
        else:
            ref_actions = np.concatenate((self.action_sequence[self.ref_ind_cur: self.action_sequence.shape[0]],
                                          np.zeros(
                                              self.prediction_len - (self.action_sequence.shape[0] - self.ref_ind_cur),
                                              dtype=np.int)))

        return ref_actions

    @staticmethod
    def get_b_spline(action_sequence, degree):
        quad_pose = np.array([0.0, 0.0, 1.0])
        quad_path = np.zeros((np.shape(action_sequence)[0] + 1, 3))
        quad_path[0, :] = quad_pose

        for it_actions in range(np.shape(action_sequence)[0]):
            quad_pose[0:2] += DroneEnv.action_2_d_path(action_sequence[it_actions])
            quad_path[it_actions + 1, :] = quad_pose

        quad_path = np.array(quad_path)
        count = quad_path.shape[0]
        knots = np.clip(np.arange(count + degree + 1) - degree, 0, count - degree)

        spl = si.BSpline(knots, quad_path, degree)
        max_t = count - degree

        # search closest points on spline
        t_search = 0.0
        dt = float(max_t)/(np.shape(action_sequence)[0])
        spl_projections = np.zeros((np.shape(quad_path)[0], 3))

        for it_points in range(np.shape(quad_path)[0]):
            search_range = np.linspace(t_search, t_search + 2.0*dt, 200)
            search_range = np.clip(search_range, 0.0, max_t)

            t_poses = spl(search_range)
            distances = np.linalg.norm(t_poses - quad_path[it_points, :], axis=1)
            min_indx = np.argmin(distances)
            t_search = search_range[min_indx]
            spl_point = spl(t_search)
            spl_projections[it_points, :] = spl_point

        return spl, quad_path, spl_projections, max_t

    @staticmethod
    def action_2_d_path(action):
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