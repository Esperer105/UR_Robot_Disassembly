# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
import geometry_msgs.msg
import tf
import math

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
        self.A = np.diag([1,1,1,1,1,1,1])
        self.B = 0
        self.U1 = 0
        self.R = np.diag([0.01,0.01,0.01,0.2,0.2,0.2,0.2])
        self.P0=np.diag([0.05,0.05,0.05,1,1,1,1])
        self.Q=np.diag([0,0,0,0,0,0,0])
        self.H = np.diag([1,1,1,1,1,1,1])
        self.P1_list=[self.P0]
        self.X1_np=np.array([0,0,0,0,0,0,0]).reshape([7,1])
        self.Z1_np=np.array([0,0,0,0,0,0,0]).reshape([7,1])



    def kf_predict(self,Xi,Pi):
        X10 = np.dot(self.A, Xi) + np.dot(self.B, self.U1)
        P10 = np.dot(np.dot(self.A, Pi), self.A.T) + self.Q
        return (X10, P10)


    def kf_update(self,X10, P10, Z):
        V = Z - np.dot(self.H, X10)
        K = np.dot(np.dot(P10, self.H.T), np.linalg.pinv(np.dot(np.dot(self.H, P10), self.H.T) + self.R))
        X1 = X10 + np.dot(K, V)
        Sqrt=math.sqrt(pow(X1[3,0],2)+pow(X1[4,0],2)+pow(X1[5,0],2)+pow(X1[6,0],2))
        for i in range(4):
            X1[i+3,0]=X1[i+3,0]/Sqrt
        P1 = np.dot(1 - np.dot(K, self.H), P10)
        return (X1, P1)
    
    def iteration(self,pose):
        s=self.itr_sum
        x=pose.position.x
        y=pose.position.y
        z=pose.position.z
        xx=pose.orientation.x
        yy=pose.orientation.y
        zz=pose.orientation.z
        w=pose.orientation.w
        Zi=np.array([x, y, z,xx,yy,zz,w]).reshape([7, 1])
        if (s==0):
            self.X1_np= np.copy(Zi)
            self.Z1_np= np.copy(Zi)
            self.itr_time+= 1
            self.itr_sum+= 1
            return Zi
        else:
            Xi = self.X1_np[:, s - 1].reshape([7, 1])
            Pi = self.P1_list[s - 1]
            X10, P10 = self.kf_predict(Xi, Pi)
            X1, P1=self. kf_update(X10, P10, Zi)
            self.Z1_np = np.concatenate([self.Z1_np, Zi], axis=1)
            self.X1_np = np.concatenate([self.X1_np, X1], axis=1)
            self.P1_list.append(P1)
            self.itr_time+= 1
            self.itr_sum+= 1
            if(self.itr_time==self.itr_num):
                self.finished=True
            else:
                dist=math.sqrt(pow(X1[0,:]-Xi[0,:],2)+pow(X1[1,:]-Xi[1,:],2)+pow(X1[2,:]-Xi[2,:],2))
                if (dist<0.005):
                    self.finished=True
            return X1

    def reset(self):
        self.finished=False
        self.itr_time=0
    
    def plot(self):
        if (self.itr_time!=0):
            fig = plt.figure()
            ax1 = fig.add_subplot(3, 1, 1)
            ax2 = fig.add_subplot(3, 1, 2)
            ax3 = fig.add_subplot(3, 1, 3)
            # ax4 = fig.add_subplot(7, 1, 4)
            # ax5 = fig.add_subplot(7, 1, 5)
            # ax6 = fig.add_subplot(7, 1, 6)
            # ax7 = fig.add_subplot(7, 1, 7)

            print('plot is working')

            ax1.plot(self.X1_np[0, :], 'go--', label="x_Kalman Filter")
            ax2.plot(self.X1_np[1, :], 'go--', label="y_Kalman Filter")
            ax3.plot(self.X1_np[2, :], 'go--', label="z_Kalman Filter")
            # ax4.plot(self.X1_np[3, :], 'go--', label="xx_Kalman Filter")
            # ax5.plot(self.X1_np[4, :], 'go--', label="yy_Kalman Filter")
            # ax6.plot(self.X1_np[5, :], 'go--', label="zz_Kalman Filter")
            # ax7.plot(self.X1_np[6, :], 'go--', label="w_Kalman Filter")

            step=self.itr_sum
            ax1.scatter(np.arange(step), self.Z1_np[0,:].reshape(step,1), label="x_Observation", marker='*')
            ax2.scatter(np.arange(step), self.Z1_np[1,:].reshape(step,1), label="y_Observation", marker='*')
            ax3.scatter(np.arange(step), self.Z1_np[2,:].reshape(step,1), label="z_Observation", marker='*')
            # ax4.scatter(np.arange(step), self.Z1_np[3,:].reshape(step,1), label="xx_Observation", marker='*')        
            # ax5.scatter(np.arange(step), self.Z1_np[4,:].reshape(step,1), label="yy_Observation", marker='*')
            # ax6.scatter(np.arange(step), self.Z1_np[5,:].reshape(step,1), label="zz_Observation", marker='*')
            # ax7.scatter(np.arange(step), self.Z1_np[6,:].reshape(step,1), label="w_Observation", marker='*')
             
            print(plt.show())
