# -*- coding: utf-8 -*-

import numpy as np
# import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as sci_R

import geometry_msgs.msg
import tf
import math
import eigenpy

class Kalman:

    """
    X(k) = AX(k-1) + BU(k) + w(k-1)
    Z(k) = HX(k) + e(k)
    p(w) = N(0, Q)
    p(e) = N(0, R)
    """
    def __init__(self, itr_num_):
        self.itr_num=itr_num_
        self.itr_time=0
        self.itr_sum=0
        self.finished=False
        self.plt_time=0
        # self.A = np.diag([1,1,1,1,1,1,1])
        self.A=np.diag([1,1,1,1,1,1])
        self.B = 0
        self.U1 = 0
        # self.R = np.diag([0.01,0.01,0.01,0.2,0.2,0.2,0.2])
        self.R = np.diag([0.01,0.01,0.01,0.2,0.2,0.2])
        # self.P0=np.diag([0.05,0.05,0.05,1,1,1,1])
        self.P0=np.diag([0.05,0.05,0.05,1,1,1])
        # self.Q=np.diag([0,0,0,0,0,0,0])
        self.Q=np.diag([0,0,0,0,0,0])
        # self.H = np.diag([1,1,1,1,1,1,1])
        self.H = np.diag([1,1,1,1,1,1])
        self.P1_list=[self.P0]
        # self.X1_np=np.array([0,0,0,0,0,0,0]).reshape([7,1])
        # self.Z1_np=np.array([0,0,0,0,0,0,0]).reshape([7,1])
        self.X1_np=np.array([0,0,0,0,0,0]).reshape([6,1])
        self.Z1_np=np.array([0,0,0,0,0,0]).reshape([6,1])



    def kf_predict(self,Xi,Pi):
        X10 = np.dot(self.A, Xi) + np.dot(self.B, self.U1)
        P10 = np.dot(np.dot(self.A, Pi), self.A.T) + self.Q
        return (X10, P10)


    def kf_update(self,X10, P10, Z):
        V = Z - np.dot(self.H, X10)
        K = np.dot(np.dot(P10, self.H.T), np.linalg.pinv(np.dot(np.dot(self.H, P10), self.H.T) + self.R))
        X1 = X10 + np.dot(K, V)
        Sqrt=math.sqrt(pow(X1[3,0],2)+pow(X1[4,0],2)+pow(X1[5,0],2))
        for i in range(3):
            X1[i+3,0]=X1[i+3,0]/Sqrt
        P1 = np.dot(1 - np.dot(K, self.H), P10)
        return (X1, P1)
    
    def iteration(self,pose):
        s=self.itr_sum
        x=pose.position.x
        y=pose.position.y
        z=pose.position.z
        r=sci_R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        e=np.dot(r.as_dcm(),np.array([0,0,1]).reshape([3,1]))
        xx=e[0,0]
        yy=e[1,0]
        zz=e[2,0]
        # Zi=np.array([x, y, z,xx,yy,zz,w]).reshape([7, 1])
        Zi=np.array([x, y, z,xx,yy,zz]).reshape([6, 1])
        print(xx,yy,zz)
        print ("the %d-th iteration"%(s))
        if (s==0):
            self.X1_np= np.copy(Zi)
            self.Z1_np= np.copy(Zi)
            self.itr_time+= 1
            self.itr_sum+= 1
            Z1_pose = geometry_msgs.msg.Pose()
            Z1_pose.position.x = Zi[0,0]
            Z1_pose.position.y = Zi[1,0]
            Z1_pose.position.z = Zi[2,0]
            Z1_vector=Zi[[3,4,5],:].reshape([1,3])
            Z1_quat=(self.transform_vector(Z1_vector)).reshape([1,4])
            # q = tf.transformations.quaternion_from_euler(-math.pi, 0, 0.5*math.pi)
            Z1_pose.orientation.x = Z1_quat[0,0]
            Z1_pose.orientation.y = Z1_quat[0,1]
            Z1_pose.orientation.z = Z1_quat[0,2]
            Z1_pose.orientation.w= Z1_quat[0,3]
            return Z1_pose
        else:
            # Xi = self.X1_np[:, s - 1].reshape([7, 1])
            Xi = self.X1_np[:, s - 1].reshape([6, 1])
            Pi = self.P1_list[s - 1]
            X10, P10 = self.kf_predict(Xi, Pi)
            X1, P1=self. kf_update(X10, P10, Zi)
            self.Z1_np = np.concatenate([self.Z1_np, Zi], axis=1)
            self.X1_np = np.concatenate([self.X1_np, X1], axis=1)
            self.P1_list.append(P1)
            self.itr_time+= 1
            self.itr_sum+= 1
            X1_vector=X1[[3,4,5],:].reshape([1,3])
            Xi_vector=Xi[[3,4,5],:].reshape([3,1])
            X1_quat=(self.transform_vector(X1_vector)).reshape([1,4])
            # Xi_quat=(self.transform_vector(Xi_vector)).reshape([1,4])
            if(self.itr_time==self.itr_num):
                self.finished=True
            else:
                dist=math.sqrt(pow(X1[0,0]-Xi[0,0],2)+pow(X1[1,0]-Xi[1,0],2)+pow(X1[2,0]-Xi[2,0],2))
                # (r, p, y) = tf.transformations.euler_from_quaternion([X1[3,0], X1[4,0], X1[5,0], X1[6,0]])
                # (x1,y1,z1,w1)= (-Xi_quat[0,0], -Xi_quat[0,1], -Xi_quat[0,2], Xi_quat[0,3])
                # (x2,y2,z2,w2)=(X1_quat[0,0], X1_quat[0,1], X1_quat[0,2], X1_quat[0,3])
                # ang_dist=2*np.arccos(w1*w2-x1*x2-y1*y2-z1*z2)
                ang_mat=np.dot(X1_vector,Xi_vector)
                ang_dist=ang_mat[0,0]
                print('ang_dist:')
                print(ang_dist)
                if (dist<0.001) and (ang_dist>0.996):
                    self.finished=True
            X1_pose = geometry_msgs.msg.Pose()
            X1_pose.position.x = X1[0,0]
            X1_pose.position.y = X1[1,0]
            X1_pose.position.z = X1[2,0]
            X1_pose.orientation.x = X1_quat[0,0]
            X1_pose.orientation.y = X1_quat[0,1]
            X1_pose.orientation.z = X1_quat[0,2]
            X1_pose.orientation.w= X1_quat[0,3]
            return X1_pose

    def transform_vector(self,vector):
        # print('n_vector:', n_vector)
        c_vector1 = np.cross(vector, [[-1, 0, 0]])
        nc_vector1=c_vector1/np.linalg.norm(c_vector1)
        # print('cross vector1:', cross_vector1)
        c_vector2 = np.cross(vector, c_vector1)
        nc_vector2=c_vector2/np.linalg.norm(c_vector2)
        # print('cross vector2:', cross_vector2)
        R_mat = np.vstack((np.vstack((nc_vector1, nc_vector2)), vector))
        R_mat = np.linalg.inv(R_mat)
        sci_r = sci_R.from_dcm(R_mat)
        return sci_r.as_quat()

    def get_former_pose(self):
        s=self.itr_sum
        # Xi=self.X1_np[:, s - 1].reshape([7, 1])
        Xi=self.X1_np[:, s - 1].reshape([6, 1])        
        Xi_pose = geometry_msgs.msg.Pose()
        Xi_pose.position.x = Xi[0,0]
        Xi_pose.position.y = Xi[1,0]
        Xi_pose.position.z = Xi[2,0]
        # Xi_pose.orientation.x = Xi[3,0]
        # Xi_pose.orientation.y = Xi[4,0]
        # Xi_pose.orientation.z = Xi[5,0]
        # Xi_pose.orientation.w= Xi[6,0]
        Xi_vector=Xi[[3,4,5],:].reshape([1,3])
        Xi_quat=(self.transform_vector(Xi_vector)).reshape([1,4])
        # q = tf.transformations.quaternion_from_euler(-math.pi, 0, 0.5*math.pi)
        Xi_pose.orientation.x = Xi_quat[0,0]
        Xi_pose.orientation.y = Xi_quat[0,1]
        Xi_pose.orientation.z = Xi_quat[0,2]
        Xi_pose.orientation.w= Xi_quat[0,3]
        return Xi_pose

    def reset(self):
        self.finished=False
        self.itr_time=0
    
    def release(self):
        del self


    def plot(self):
        if (self.itr_time!=0):
            fig = plt.figure(figsize=(16,9)  , dpi=120)

            # ax1 = fig.add_subplot(7, 1, 1)
            # ax2 = fig.add_subplot(7, 1, 2)
            # ax3 = fig.add_subplot(7, 1, 3)
            # ax4 = fig.add_subplot(7, 1, 4)
            # ax5 = fig.add_subplot(7, 1, 5)
            # ax6 = fig.add_subplot(7, 1, 6)
            # ax7 = fig.add_subplot(7, 1, 7)
            ax1 = fig.add_subplot(6, 1, 1)
            ax2 = fig.add_subplot(6, 1, 2)
            ax3 = fig.add_subplot(6, 1, 3)
            ax4 = fig.add_subplot(6, 1, 4)
            ax5 = fig.add_subplot(6, 1, 5)
            ax6 = fig.add_subplot(6, 1, 6)
            print('plot is working')

            ax1.plot(self.X1_np[0, :], 'go--', label="x_Kalman Filter")
            ax2.plot(self.X1_np[1, :], 'go--', label="y_Kalman Filter")
            ax3.plot(self.X1_np[2, :], 'go--', label="z_Kalman Filter")
            ax4.plot(self.X1_np[3, :], 'go--', label="xx_Kalman Filter")
            ax5.plot(self.X1_np[4, :], 'go--', label="yy_Kalman Filter")
            ax6.plot(self.X1_np[5, :], 'go--', label="zz_Kalman Filter")
            # ax7.plot(self.X1_np[6, :], 'go--', label="w_Kalman Filter")

            step=self.itr_sum
            ax1.scatter(np.arange(step), self.Z1_np[0,:].reshape(step,1), label="x_Observation", marker='o')
            ax1.legend()
            ax2.scatter(np.arange(step), self.Z1_np[1,:].reshape(step,1), label="y_Observation", marker='o')
            ax2.legend()
            ax3.scatter(np.arange(step), self.Z1_np[2,:].reshape(step,1), label="z_Observation", marker='o')
            ax3.legend()
            ax4.scatter(np.arange(step), self.Z1_np[3,:].reshape(step,1), label="xx_Observation", marker='o') 
            ax4.legend()
            ax5.scatter(np.arange(step), self.Z1_np[4,:].reshape(step,1), label="yy_Observation", marker='o')
            ax5.legend()
            ax6.scatter(np.arange(step), self.Z1_np[5,:].reshape(step,1), label="zz_Observation", marker='o')
            ax6.legend()
            # ax7.scatter(np.arange(step), self.Z1_np[6,:].reshape(step,1), label="w_Observation", marker='*')
            plt.legend()
            plt.suptitle('Result of Kalman Filter')            
            plt.savefig("kalman_"+str(self.plt_time+1)+".png")
            self.plt_time+=1
            print(plt.show())