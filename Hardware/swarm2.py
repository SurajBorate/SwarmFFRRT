# rostopic pub /burger/reset std_msgs/Empty "{}"
import os
from bezier import find_bezier_trajectory
from cubictraj import find_cubic_traj, find_xy_cubic
# os.system('rostopiv')
#print('Resetting odom ... Wait for 5 seconds and then press Ctrl C.')
# os.system('rostopic pub /burger/reset std_msgs/Empty "{}"')
# os.system('rostopic pub /waffle_pi/reset std_msgs/Empty "{}"')
os.system('rostopic pub /firebird/reset std_msgs/Empty "{}"')

import rospy
import numpy as np
from turtlebot import turtlebot3
import time
# burger','waffle_pi',

class Swarm():
    def __init__(self,bots=['firebird']):
        rospy.init_node('swarm')
        self.botSwarm = []
        for i in range(len(bots)):
        # waffle = turtlebot3('waffle_pi')
        # waffle.PIDController.K_th = 1
        # self.botSwarm.append(waffle)
            self.botSwarm.append(turtlebot3(bots[i]))

        self.botSwarm[0].PIDController.K_th = 3
        #self.botSwarm[0].PIDController.K_st = 1
    def move(self,):
        target = 1
        rate = rospy.Rate(10.0)
        
        j=1
        # pts = [[0,0],[0,0.25],[0,0.5],[0,0.75],
        #        [0,1],[0.25,0],[0.5,1],[0.75,1],
        #        [1,1],[1,0.75],[1,0.5],[1,0.25],
        #        [1,0]]
        # pts1 = [[0,0],[0.06,0.4],[0.12,0.4],[2,0]]
        # pts2 = [[0,0],[0.06,-0.4],[0.12,-0.4],[2,0]]
        # pts3 = [[0,0],[0.06,0],[0.12,0],[2,0]]

        # pts1 = np.array([[0,0],[1,0],[1,1],[0,1]])
        # pts2 = np.array([[0,0],[1,0],[1,1],[0,1]])
        # pts3 = np.array([[0,0],[1,0],[1,1],[0,1]])
        ways = np.load('way.npy')
        print(ways)
        
        wnn = np.shape(ways)
        wnn = wnn[0]
        waypoints_r1 = np.zeros((wnn,2))
        waypoints_r2 = np.zeros((wnn,2))
        for i in range(wnn):
            waypoints_r1[i][:] = ways[i][0]/1000
            waypoints_r2[i][:] = ways[i][1]/1000

        t=np.linspace(0,1,100)
        t0 = 0
        tf = 30
        V = 0.1
        dt = 0.001
       
        Px1,Py1,Tf = find_cubic_traj(waypoints_r1,t0,tf,V,dt)
        # Px2,Py2,Tf = find_cubic_traj(waypoints_r2,t0,tf,V,dt)
        #Px3,Py3,Tf = find_cubic_traj(pts3,t0,tf,V,dt)
        
        
        
        # exit()
        
        npt = np.shape(waypoints_r2)
        npt = npt[0]-1
        
        k = 0
        st_time = time.time()
        while not rospy.is_shutdown():
            while k < npt:
                if time.time() - st_time<Tf[k]:
                    x1,y1 = find_xy_cubic(Px1[k],Py1[k],time.time() - st_time)
                    self.botSwarm[0].move([x1,y1])
                    # x2,y2 = find_xy_cubic(Px2[k],Py2[k],time.time() - st_time)
                    # self.botSwarm[1].move([x2,y2])
                    #x3,y3 = find_xy_cubic(Px3[k],Py3[k],time.time() - st_time)
                    #self.botSwarm[2].move([x3,y3])
                else:
                    k = k + 1
            rate.sleep()

a = Swarm()
a.move()
