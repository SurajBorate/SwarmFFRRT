import numpy as np
from scipy.spatial.transform import Rotation as R

class PID_Controller():
    def __init__(self,gains=[[0.8,0.0,0.5,0],
                             [1.4,0.0,0,0.0]]):

        self.Gains = np.array(gains)
        self.error_buffer = np.zeros((len(gains),5))

        self.vels = np.array([0,0])


    def sum_errors(self):
        error_sum = []
    
        for i in range(len(self.error_buffer)):
            error_sum.append(np.sum(self.error_buffer))

        return np.array(error_sum)

    # Anti wind up not added yet
    def track(self,error,bot_quat):



        ###############
        # Will be used in Turtlebot to convert quaternions to required heading angle
        quat = R.from_quat(bot_quat)
        heading_angle = (quat.as_euler('zyx',degrees=False)[-1])
        if heading_angle<0:
            heading_angle = -(heading_angle+np.pi)
        else:
            heading_angle = -(heading_angle - np.pi)
        ################


        param_ang_error = np.arctan2(error[1],error[0]) - heading_angle  
        # print(param_ang_error)
        param = -np.tanh(2*(param_ang_error-np.pi/2))*np.tanh(2*(param_ang_error+np.pi/2))
        # print(param)
        dist_error = param*np.linalg.norm(error) 

        ang_error = np.arctan2(error[1],error[0]) - heading_angle
        if ang_error>np.pi/2:
            ang_error = ang_error - np.pi
        elif ang_error<-np.pi/2:
            ang_error = ang_error + np.pi

        curr_error = np.array([dist_error,ang_error])
        self.error_buffer = np.hstack((self.error_buffer[:,1:],np.transpose([curr_error])))

        prop  = self.Gains[:,0]*curr_error
        intgr = self.Gains[:,1]*self.sum_errors()
        deri  = self.Gains[:,2]*(self.error_buffer[:,-1] - self.error_buffer[:,-2])

        damp  = self.Gains[:,3]*self.vels

        outputs = prop + intgr + deri - damp
        return outputs

