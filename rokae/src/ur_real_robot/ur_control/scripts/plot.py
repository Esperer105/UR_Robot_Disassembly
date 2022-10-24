# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt


X1_np=np.loadtxt("/home/zys/UR_Robot_Disassembly/rokae/data/pose_estimation/RANSAC_KF/filt_10.csv",delimiter=",")
Z1_np=np.loadtxt("/home/zys/UR_Robot_Disassembly/rokae/data/pose_estimation/RANSAC_KF/obs_10.csv",delimiter=",")


fig = plt.figure(figsize=(16,9)  , dpi=120)


# ax1 = fig.add_subplot(6, 1, 1)
# ax2 = fig.add_subplot(6, 1, 2)
# ax3 = fig.add_subplot(6, 1, 3)
# ax4 = fig.add_subplot(6, 1, 4)
# ax5 = fig.add_subplot(6, 1, 5)
# ax6 = fig.add_subplot(6, 1, 6)
ax1 = fig.add_subplot(3, 1, 1)
ax2 = fig.add_subplot(3, 1, 2)
ax3 = fig.add_subplot(3, 1, 3)


print('plot is working')

ax1.plot(X1_np[0, :], 'go--', label="x_Kalman Filter")
ax2.plot(X1_np[1, :], 'go--', label="y_Kalman Filter")
ax3.plot(X1_np[2, :], 'go--', label="z_Kalman Filter")
# ax4.plot(X1_np[3, :], 'go--', label="xx_Kalman Filter")
# ax5.plot(X1_np[4, :], 'go--', label="yy_Kalman Filter")
# ax6.plot(X1_np[5, :], 'go--', label="zz_Kalman Filter")


step=X1_np.shape[1]
ax1.scatter(np.arange(step), Z1_np[0,:].reshape(step,1), label="x_Observation", marker='o')
ax1.legend()
ax2.scatter(np.arange(step), Z1_np[1,:].reshape(step,1), label="y_Observation", marker='o')
ax2.legend()
ax3.scatter(np.arange(step), Z1_np[2,:].reshape(step,1), label="z_Observation", marker='o')
ax3.legend()
# ax4.scatter(np.arange(step), Z1_np[3,:].reshape(step,1), label="xx_Observation", marker='o') 
# ax4.legend()
# ax5.scatter(np.arange(step), Z1_np[4,:].reshape(step,1), label="yy_Observation", marker='o')
# ax5.legend()
# ax6.scatter(np.arange(step), Z1_np[5,:].reshape(step,1), label="zz_Observation", marker='o')
# ax6.legend()


plt.legend()
plt.suptitle('Result of Kalman Filter')            
plt.savefig("kalman.png")
print(plt.show())