# rostopic pub /burger/reset std_msgs/Empty "{}"
import os
from bezier import find_bezier_trajectory
from cubictraj import find_cubic_traj, find_xy_cubic#,find_cubic_traj_new, find_xy_cubic_new
from matplotlib import pyplot as plt
# os.system('rostopiv')
#print('Resetting odom ... Wait for 5 seconds and then press Ctrl C.')
# os.system('rostopic pub /burger/reset std_msgs/Empty "{}"')
# os.system('rostopic pub /waffle_pi/reset std_msgs/Empty "{}"')
# os.system('rostopic pub /waffle/reset std_msgs/Empty "{}"')
os.system('rostopic pub /burger_1/reset std_msgs/Empty "{}"')
# os.system('rostopic pub /burger_2/reset std_msgs/Empty "{}"')
# os.system('rostopic pub /burger_3/reset std_msgs/Empty "{}"')

import rospy
import numpy as np
from turtlebot import turtlebot3
import time
# burger','waffle_pi',

class Swarm():
    def __init__(self,bots=['burger_3']):
    # def __init__(self,bots=['burger_3','waffle','burger_1','burger_2']):
        rospy.init_node('swarm')
        self.botSwarm = []
        for i in range(len(bots)):
        # waffle = turtlebot3('waffle_pi')
        # waffle.PIDController.K_th = 1
        # self.botSwarm.append(waffle)
            self.botSwarm.append(turtlebot3(bots[i]))
        gains_burger3 = [1,2,-0.1,0]
        gains_waffle = [1,2,-0.1,0]
        gains_burger1 = [1,2,-0.1,0]
        gains_burger2 = [1,2,-0.1,0]
        # self.botSwarm[0].PIDController.Gains = np.array(gains_burger3)
        # self.botSwarm[1].PIDController.Gains = np.array(gains_waffle)
        self.botSwarm[0].PIDController.Gains = np.array(gains_burger1)
        # self.botSwarm[3].PIDController.Gains = np.array(gains_burger2)
        self.botSwarm[0].PIDController.K_st = 1

    def move(self,):
        target = 1
        rate = rospy.Rate(1.0)
        
        j=1

        ways = np.load('locs.npy')
        wnn = np.shape(ways)
        wnn = wnn[0]
        waypoints_r1 = np.zeros((wnn,2))
        waypoints_r2 = np.zeros((wnn,2))
        waypoints_r3 = np.zeros((wnn,2))
        waypoints_r4 = np.zeros((wnn,2))
        
        for i in range(wnn):
            waypoints_r1[i][:] = ways[i][0]
            waypoints_r2[i][:] = ways[i][1]
            waypoints_r3[i][:] = ways[i][2]
            waypoints_r4[i][:] = ways[i][3]

        t=np.linspace(0,1,100)
        t0 = 0
        tf = 30
        V = 0.1
        dt = 0.01

        waypoints = np.array([[0,0],[-1,0],[-1,1],[0,1]])
        
        Px1,Py1,Tf = find_cubic_traj(waypoints, t0, tf, V, dt)
        Px2,Py2,Tf = find_cubic_traj(waypoints, t0, tf, V, dt)
        Px3,Py3,Tf = find_cubic_traj(waypoints, t0, tf, V, dt)
        Px4,Py4,Tf = find_cubic_traj(waypoints, t0, tf, V, dt)
        
        
        # exit()
        
        npt = np.shape(Tf)
        npt = npt[0]

        k = 0
        
        st_time = time.time()
        while not rospy.is_shutdown():
            while k < npt:
                print(k)
                print(time.time()-st_time)
                if ((time.time() - st_time)<Tf[k]):
                    x1,y1 = find_xy_cubic(Px1[k],Py1[k],time.time() - st_time)                   
                    self.botSwarm[0].move([x1,y1])
                else: k = k + 1


            rate.sleep()

a = Swarm()
a.move()
