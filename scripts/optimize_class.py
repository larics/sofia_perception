#!/usr/bin/env python3

import rospkg
import numpy as np
import scipy.optimize
import scipy
import scipy.io
from tf.transformations import quaternion_matrix

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class Fminsearch:
    def load_set(self, points, tfs):
        '''Loads detected blob positions and corresponding transformations T_0^f from .txt files.

        Arguments:
        points - path to .txt file containing blob positions in the camera frame
        tfs - path to .txt file containing transformations from the robot base frame to the robot flange frame'''

        self.data = np.loadtxt(points, delimiter=",")

        # homogeneous coordinates
        data_pos = []
        for row in self.data:
            new_row = []
            for i in row:
                new_row.append(i)
            new_row.append(1.)
            data_pos.append(np.asarray(new_row))
        self.data = np.asmatrix(data_pos)

        tf = []
        vec = []
        with open(tfs, "r+") as f:
            lines = f.readlines()
            for line in lines:
                if "translation" in line:
                    if len(vec) > 0:
                        tf.append(vec)
                    vec = []
                elif "rotation" in line:
                    pass
                else:
                    val = line.split(':')[1]
                    vec.append(float(val))

        tf.append(vec)
        self.tfs = np.asmatrix(tf)
        print("Data points: ")
        print(self.data)
        print("\nTransformations: ")
        print(self.tfs)

    def do_optimise_T(self, t1):
        '''Runs optimization algorithm.
        
        Arguments:
        t1 - initial guess for the optimization'''

        def objectiveFunLambda(x): return self.objectiveFunTransformation(
            x, self.data, self.tfs)
        topt = scipy.optimize.fmin(func=objectiveFunLambda, x0=t1)

        q = topt[3:]
        q /= np.linalg.norm(q)
        topt[3:] = q
        print("\nOptimized transform: " + str(topt))

        return topt, self.objectiveFunTransformation(topt, self.data, self.tfs)

    def objectiveFunTransformation(self, posquat, data_pos, data_tf):
        '''Objective function - function whose value is to be minimized.
        
        Arguments:
        posquat - transformation from the flange frame to the camera frame
        data_pos - blob positions in the camera frame
        data_tf - transformations from the robot base frame to the flange frame
        
        Returns:
        f - function value which is being minimized'''

        # Tx - flange-camera transformation matrix
        Tx = np.eye(4, dtype="float32")

        Tx = np.asmatrix(quaternion_matrix(posquat[3:]))
        Tx[0:3, 3] = np.resize(posquat[0:3], (3, 1))

        # Teef - base-flange transformation matrices
        [l, m] = np.shape(data_tf)
        Teef = np.zeros((l, 4, 4))
        for i in range(l):
            Teef[i, :, :] = np.asmatrix(
                quaternion_matrix(np.asarray(data_tf)[i, 3:]))
            Teef[i, 0:3, 3] = np.asarray(data_tf)[i, 0:3]

        #########################
        ## YOUR CODE GOES HERE ##
        #########################
        # return f

    def transformInputData(self, posquat):
        '''Transforms input data using provided transformation.
        
        Arguments:
        posquat - transformation from the flange frame to the camera frame'''

        self.txs_optimized = np.zeros_like(self.data)

        Tx = np.asmatrix(quaternion_matrix(posquat[3:]))
        Tx[0:3, 3] = np.resize(posquat[0:3], (3, 1))

        [l, m] = np.shape(self.tfs)
        Teef = np.zeros((l, 4, 4))
        for i in range(l):
            Teef[i, :, :] = np.asmatrix(
                quaternion_matrix(np.asarray(self.tfs)[i, 3:]))
            Teef[i, 0:3, 3] = np.asarray(self.tfs)[i, 0:3]

        [l, m] = np.shape(self.data)

        for i in range(l):
            Tglob = np.dot(Teef[i, :, :], Tx)
            glob_1 = np.dot(Tglob, np.transpose(self.data[i, :]))
            self.txs_optimized[i, 0:3] = np.transpose(glob_1[0:3])

        print(self.txs_optimized)

    def plot_inputs(self, ax1, col='k'):
        '''Plots input data transformed using optimized transformation.'''

        mtrx = self.txs_optimized.transpose()
        pos_vec = []
        m, n = np.shape(mtrx)
        print(m, n)
        for i in range(n):
            m = mtrx[0:3, i]
            pos_vec.append(m)

        cm = np.asarray(pos_vec)
        ax1.scatter(cm[:, 0], cm[:, 1], cm[:, 2], color=col)


if __name__ == "__main__":
    rospack = rospkg.RosPack()
    poses_path = rospack.get_path('sofia_perception') + '/resources/poses.txt'
    tfs_path = rospack.get_path('sofia_perception') + '/resources/tfs.txt'

    optim = Fminsearch()
    optim.load_set(poses_path, tfs_path)

    # initial guess for the optimization algorithm written as position + quaternion (x, y, z, qx, qy, qz, qw)
    t1 = np.asarray([0, 0, 0, 0, 0, 0, 0])
    topt, f = optim.do_optimise_T(t1)

    print("dissipation with T used:")
    print(optim.objectiveFunTransformation(t1, optim.data, optim.tfs))
    print("dissipation with T optimized:")
    print(optim.objectiveFunTransformation(topt, optim.data, optim.tfs))

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    optim.transformInputData(topt)
    optim.plot_inputs(ax)
    plt.show()
