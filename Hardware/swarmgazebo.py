# rostopic pub /burger/reset std_msgs/Empty "{}"
import os
from bezier import find_bezier_trajectory
from cubictraj import find_cubic_traj, find_xy_cubic, parameter_estimate#,find_cubic_traj_new, find_xy_cubic_new
from matplotlib import pyplot as plt
from movingaveragesmoothing import *
# os.system('rostopiv')
#print('Resetting odom ... Wait for 5 seconds and then press Ctrl C.')
# os.system('rostopic pub /burger/reset std_msgs/Empty "{}"')
# os.system('rostopic pub /waffle_pi/reset std_msgs/Empty "{}"')
os.system('rostopic pub /tb3_0/reset std_msgs/Empty "{}"')
os.system('rostopic pub /tb3_1/reset std_msgs/Empty "{}"')
os.system('rostopic pub /tb3_2/reset std_msgs/Empty "{}"')
os.system('rostopic pub /tb3_3/reset std_msgs/Empty "{}"')


import rospy
import numpy as np
from turtlebot import turtlebot3
import time
# burger','waffle_pi',

class Swarm():
    #def __init__(self,bots=['burger_1']):
    def __init__(self,bots=['tb3_0','tb3_1','tb3_2','tb3_3']):
        self.botSwarm = []
        for i in range(len(bots)):
            self.botSwarm.append(turtlebot3(bots[i]))
        gains_burger3 = [0.1,0,0,0]
        gains_waffle = [0.1,0,0,0]
        gains_burger1 = [0.1,0,0,0]
        gains_burger2 = [0.1,0,0,0]
        self.botSwarm[0].PIDController.Gains = np.array(gains_burger3)
        self.botSwarm[1].PIDController.Gains = np.array(gains_waffle)
        self.botSwarm[2].PIDController.Gains = np.array(gains_burger1)
        self.botSwarm[3].PIDController.Gains = np.array(gains_burger2)

    def move(self,):
        
        # pts = [[0,0],[0,0.25],[0,0.5],[0,0.75],
        #        [0,1],[0.25,0],[0.5,1],[0.75,1],
        #        [1,1],[1,0.75],[1,0.5],[1,0.25],
        #        [1,0]]
        # pts1 = [[0,0],[0.06,0.4],[0.12,0.4],[2,0]]
        # pts2 = [[0,0],[0.06,-0.4],[0.12,-0.4],[2,0]]
        # pts3 = [[0,0],[0.06,0],[0.12,0],[2,0]]

        # pts1 = np.array([[0,0],[1,0],[1,1],[0,1]])
        # pts2 = np.array([[0,0],[1,0],[1,1],[0,1]])
        pts3 = np.array([[0,0],[1,0],[2,0]])
        #pts3 = np.array([[0,0],[-0.2,0],[-0.2,0.2]])
        ways = np.load('locs1.npy')
        wnn = np.shape(ways)
        wnn = wnn[0]
        waypoints_r1 = np.zeros((wnn,2))
        waypoints_r2 = np.zeros((wnn,2))
        # waypoints_r3 = np.zeros((wnn,2))
        # waypoints_r4 = np.zeros((wnn,2))
        
        for i in range(wnn):
            waypoints_r1[i][:] = ways[i][0]
            waypoints_r2[i][:] = ways[i][1]
            # waypoints_r3[i][:] = ways[i][2]
            # waypoints_r4[i][:] = ways[i][3]

        t=np.linspace(0,1,100)
        t0 = 0
        
        V = 0.05
        dt = 0.0001
       
        # Px1,Py1,Xtraj1,Ytraj1,t,Tf = find_cubic_traj(waypoints_r1/1000,t0,tf,V,dt)
        # Px2,Py2,Xtraj2,Ytraj2,t,Tf = find_cubic_traj(waypoints_r2/1000,t0,tf,V,dt)
        # Px3,Py3,Xtraj3,Ytraj3,t,Tf = find_cubic_traj(waypoints_r3/1000,t0,tf,V,dt)
        # Px4,Py4,Xtraj4,Ytraj4,t,Tf = find_cubic_traj(waypoints_r4/1000,t0,tf,V,dt)

        #waypoints = np.array([[0,0],[-1,0],[-1,1],[0,1]])
        spath1 = smoothen_movavg(waypoints_r1)   # heeeeere
        tf = parameter_estimate(spath1,0.1)
        waypoints1 = spath1
        waypoints1 = pts3

        spath2 = smoothen_movavg(waypoints_r2)
        tf = parameter_estimate(spath2,0.1)
        waypoints2 = spath2
        waypoints2 = pts3
        # spath3 = smoothen_movavg(waypoints_r3)
        # tf = parameter_estimate(spath3/1000,0.1)
        # waypoints3 = spath3/1000

        # spath4 = smoothen_movavg(waypoints_r4)
        # tf = parameter_estimate(spath4/1000,0.1)
        # waypoints4 = spath4/1000
        #waypoints = np.array([[1+np.cos(0),0+np.sin(0)],[1+np.cos(np.pi/6),0+np.sin(np.pi/6)],[1+np.cos(np.pi/3),0+np.sin(np.pi/3)],[1+np.cos(np.pi/2),0+np.sin(np.pi/2)]])

        # Px1,Py1,Tf = find_cubic_traj(waypoints_r1/1000, t0, tf, V, dt)
        # Px2,Py2,Tf = find_cubic_traj(waypoints_r2/1000, t0, tf, V, dt)
        # Px3,Py3,Tf = find_cubic_traj(waypoints_r3/1000, t0, tf, V, dt)
        # Px4,Py4,Tf = find_cubic_traj(waypoints_r4/1000, t0, tf, V, dt)
        # tf = 30
        Px1,Py1,Tf = find_cubic_traj(waypoints1, t0, tf, V, dt)
        Px2,Py2,Tf = find_cubic_traj(waypoints2, t0, tf, V, dt)
        print(Tf)
        # Px3,Py3,Tf = find_cubic_traj(waypoints, t0, tf, V, dt)
        # Px4,Py4,Tf = find_cubic_traj(waypoints4, t0, tf, V, dt)
        # exit()
        npt = np.shape(Tf)
        npt = npt[0]

        k = 0
        
        st_time = time.time()
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown() and k<npt:
            # while k < npt:
                # print(k)z
                # print(time.time()-st_time)
            if ((time.time() - st_time)<Tf[k]):
                x1,y1 = find_xy_cubic(Px1[k],Py1[k],time.time() - st_time)                   
                self.botSwarm[1].move([x1,y1])
                x2,y2 = find_xy_cubic(Px2[k],Py2[k],time.time() - st_time)
                self.botSwarm[2].move([x2,y2])
                x3,y3 = find_xy_cubic(Px2[k],Py2[k],time.time() - st_time)
                self.botSwarm[3].move([x3,y3])
                x4,y4 = find_xy_cubic(Px2[k],Py2[k],time.time() - st_time)
                self.botSwarm[0].move([x4,y4])
                # x4,y4 = find_xy_cubic(Px4[k],Py4[k],time.time() - st_time)
                # self.botSwarm[3].move([x4,y4])
            else: k = k + 1
                # if k == npt:
                # #         if ((self.botSwarm[0].error<0.1) and ((self.botSwarm[1].error<0.1 and self.botSwarm[2].error<0.1) and (self.botSwarm[3].error<0.1))):
                #              self.botSwarm[0].stop()
                #              self.botSwarm[1].stop()
                #              self.botSwarm[2].stop()
                #              self.botSwarm[3].stop()

            rate.sleep()

if __name__=="__main__":
    
    rospy.init_node('swarm')
    a = Swarm()
    a.move()
    rospy.spin()    
