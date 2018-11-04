
# Kalman filter

import rospy
import numpy as np

import math
from geometry_msgs.msg import TransformStamped





class kalmanPosQSingelton(object):

    def __init__(self, dt, quadID):
        self.qx = 1
        self.qy = 1
        self.qz = 1

        self.rx = 0.01
        self.ry = 0.01
        self.rz = 0.01

        self.viconPosFrameSequence = 0
        self.notVisible = 0

        self.yaw_old = 0

        self.x = np.zeros((6, 1))

        self.P = 10 * np.eye(6)
        self.H = np.concatenate((np.eye(3), np.zeros((3, 3))), axis=1)

        self.Q = np.zeros((6, 6))
        self.Q[3, 3], self.Q[4, 4], self.Q[5, 5] = self.qx, self.qy, self.qz
        self.R = np.zeros((3, 3))
        self.R[0, 0], self.R[1, 1], self.R[2, 2] = self.rx, self.ry, self.rz

        self.dt = dt


        # Ros subscriber
        self.name = 'vicon/bebop' + str(quadID) + '/bebop' + str(quadID)
        self.pos_sub = rospy.Subscriber(self.name, TransformStamped, self.vicon_cb)




    def prediction(self, dt):

        self.F = np.eye(6)
        self.F[0, 3], self.F[1, 4], self.F[2, 5] = dt, dt, dt

        self.x = np.dot(self.F, self.x)
        self.P = self.F.dot(self.P).dot(np.transpose(self.F)) + self.Q


    def update(self, z):

        y = z - self.H.dot(self.x)
        S = self.H.dot(self.P).dot(np.transpose(self.H)) + self.R

        a = self.P.dot(np.transpose(self.H))
        b = np.concatenate((S, S), axis=0)
        K = np.divide(a, b, out=np.zeros_like(a), where=b!=0)

        self.x = self.x + K.dot(y)
        self.P = (np.eye(6) - K.dot(self.H)).dot(self.P)


    def step(self, z):

        self.prediction(self.dt)
        self.update(z)

        pos = self.x[0:3]
        vel = self.x[3:7]

        return pos, vel


    def vicon_cb(self, msg):

        tmpseq = msg.header.seq
        if tmpseq > self.viconPosFrameSequence:
            self.viconPosFrameSequence = msg.header.seq
        else:
            self.notVisible = 1

        pos_raw = np.array([[msg.transform.translation.x],
                            [msg.transform.translation.y],
                            [msg.transform.translation.z]])

        q = [msg.transform.rotation.x,
             msg.transform.rotation.y,
             msg.transform.rotation.z,
             msg.transform.rotation.w]

        pos_filt, vel_filt = self.step(pos_raw)
        R_cw_real = self.RotFromQuatJ(q)

        self.yaw    = math.atan2(R_cw_real[1, 0], R_cw_real[0, 0])
        self.pitch  = math.atan2(-R_cw_real[2, 0], math.sqrt(R_cw_real[2, 1]**2 + R_cw_real[2, 2]**2))
        self.roll   = math.atan2(R_cw_real[2, 1], R_cw_real[2, 2])
        self.yqw_vel = (self.yaw - self.yaw_old) / self.dt
        self.yaw_old = self.yaw
        self.pos_filt = pos_filt
        self.vel_filt = vel_filt

        print (self.pos_filt)


    def RotFromQuatJ(self, q):
        R = np.array([[q[0]**2-q[1]**2-q[2]**2+q[3]**2, 2*(q[0]*q[1]+q[2]*q[3]), 2*(q[0]*q[2]-q[1]*q[3])],
                      [2*(q[0]*q[1]-q[2]*q[3]), -q[0]**2+q[1]**2-q[2]**2+q[3]**2, 2*(q[1]*q[2]+q[0]*q[3])],
                      [2*(q[0]*q[2]+q[1]*q[3]), 2*(q[1]*q[2]-q[0]*q[3]), -q[0]**2-q[1]**2+q[2]**2+q[3]**2]])

        return R


if __name__ == '__main__':
    rospy.init_node('test_kalmanfilter')

    rate_on = rospy.Rate(5)

    kalmanPosQSingelton(5, 103)

    while not rospy.is_shutdown():
        rate_on.sleep()
