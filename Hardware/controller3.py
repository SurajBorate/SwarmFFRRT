import numpy as np
from scipy.spatial.transform import Rotation as R

class PID_Controller():
    def __init__(self,gains=[3,8,-1.5,0]):
        self.Gains = np.array(gains)
        self.error_buffer = np.zeros((len(gains),5))

        self.vels = np.array([0,0])


    def sum_errors(self):
        error_sum = []
    
        for i in range(len(self.error_buffer)):
            error_sum.append(np.sum(self.error_buffer))

        return np.array(error_sum)

    # Anti wind up not added yet
    def track(self,error, bot_quat):
        
        quat = R.from_quat(bot_quat)
        heading_angle = (quat.as_euler('zyx',degrees=False)[-1])
        if heading_angle<0:
            heading_angle = -(heading_angle+np.pi)
        else:
            heading_angle = -(heading_angle - np.pi)


        # print(heading_angle)


        alpha = (np.arctan2(error[0], error[1]) - (np.pi/2 - heading_angle))%(2*np.pi)
        if alpha > np.pi:
            alpha = alpha - 2*np.pi
        


        beta = (np.arctan2(error[0], error[1]))%(2*np.pi)
        if beta > np.pi:
            beta = beta - 2*np.pi

        rho = np.linalg.norm(error)

        k_rho = self.Gains[0]
        k_alpha = self.Gains[1]
        k_beta = self.Gains[2]
        
        if alpha > -np.pi/2 and alpha <= np.pi/2:
            v = k_rho*rho
            omega = -(k_alpha*alpha + k_beta*beta)
        else:
            v = -k_rho*rho
            omega = k_alpha*alpha + k_beta*beta
        
        print(alpha,beta)


        return v,omega
        