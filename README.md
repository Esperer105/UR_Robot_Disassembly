# rokae_robot
ＵＲ 機器人的，運行包都在ＵＲ。


robot:            example_organization_ur_launch   ex-ur10-1.launch  
rviz:             ur10e_moveit_config   moveit_planning_execution_4_arm_world.launch




camera在 arm 的gazebo启动方式
arm_world.launch

camera在 arm 的rviz启动方式
rokae_camera_moveit_config    moveit_planning_execution.launch

单独添加电池包，battery:rokae_robot/rokae/src/rokae_control/scripts
robot_sorting.py


添加电池包并套接，battery:rokae_robot/rokae/src/rokae_control/scripts
motion_planning.py   添加电池包时，输入字符“add”，过后按照提出 输入其它字符。若是不加载电池，空一行后回车就可以啦

深度相机，我的问题是 py2  不支持touch.
src/rokae_control/scripts/bolt_position_detector.py
拆解原语
src/rokae_control/scripts/testmotion.py


无法查找功能包路径的解决办法，catkin_make ； source devel/setup.bash
https://blog.csdn.net/weizhangyjs/article/details/80521021

机器人运动规划数据样例集合，
https://python.hotexamples.com/zh/examples/moveit_commander/PlanningSceneInterface/-/python-planningsceneinterface-class-examples.html



RGBD图像转tf, 暂时需要在图像上点击一下，暂时图像检测位置点接口没有添加。src/rokae_control/scripts/bolt_position_detector.py 是螺栓的检测。src/rokae_control/scripts/battery_position.py是电池包位置的检测，里面的面积cv2.contourArea(contour) 即可改变检测范围
rgbd_imgpoint_to_tf.py


deep-learning-for-image-processing中的vgg 是图像处理 和pytorch vgg     是图像处理 是同一个东西。 教程来源https://blog.csdn.net/Action_now_zj/article/details/109250528  数据集也在这里。　脚本解释　https://zhuanlan.zhihu.com/p/279823946　　　我判断不准确是因为－过拟合



关于nsplanner模块的调用：

1. rokae仿真：

首先加载rokae_gazebo arm_world.launch，然后加载rokae_moveit_config moveit_planning_execution_4_arm_world.launch，再运行~/rokae_robot/rokae/src/rokae_control/scripts/env_setup.py，最后运行~/rokae_robot/rokae/src/rokae_control/scripts/nsplanner.py

动作原语的修改在~/rokae_robot/rokae/src/rokae_control/scripts/prim_action.py中

动作执行判据检测在~/rokae_robot/rokae/src/rokae_control/scripts/pri_cri.py中，当前为空，在nsplanner中设定默认执行结果为保守估计的结果。

2. ur10e执行：

robot:            example_organization_ur_launch   ex-ur10-1.launch
rviz:             ur10e_moveit_config   moveit_planning_execution_4_arm_world.launch
realsense:        realsense2_camera   rs_camera.launch align_depth:=true
handeye:          easy_handeye   publish.launch      需要先将/src/easy_handeye/easy_handeye/launch/ur10e_camera_handeyecalibration_eye_on_hand.yaml文件移动到/home/zys/.ros/easy_handeye目录下
nslanner:         /src/ur_real_robot/ur_control/scripts/nsplanner.py


关于bolt_detector的使用：

UR_Robot_Disassembly/rokae/src/ur_real_robot/ur_control/scripts/Realsense/Edge_Box_Model/model.yml
上述文件是Edge_box方法的模型文件，是Edge_box的必需文件

Edge_box生成预测框需要调整一些参数，参数我已调整的没有太大问题

bolt_detector的图片保存路径为rokae/src/ur_real_robot/ur_control/scripts/Realsense/images

detect_edge_box中的max_box参数越小，生成的候选框越少，运行速度越快；max_box=200,运行时间大概3s;max_box=1000,运行时间大概8s

opencv-python=4.1.2.30 opencv-contrib-python=4.1.2.30