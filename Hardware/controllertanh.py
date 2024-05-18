import numpy as np
from scipy.spatial.transform import Rotation as R
class PID_Controller():
    def __init__(self,):
        self.K_x = 0.3
        self.K_I_x = 0
        self.K_D_x = 0
        self.K_damp_x = 0

        self.K_y = 0.5
        self.K_I_y = 0 # delt will be adjusted in K_i value
        self.K_D_y = 0
        self.K_damp_y = 0

        self.v_x = 0
        self.v_y = 0

        self.del_t = 0
        self.error_buffer = []

    def sum_errors(self, type='x'):
        error_sum = 0
        flag = 0
        if type == 'y':
            flag = 1
    
        for i in range(len(self.error_buffer)):
            error_sum += self.error_buffer[i][flag]

        return error_sum
    # Anti wind up
    def track(self,error,bot_quat):
        param_ang_error = np.arctan2(error[1],error[0])  
        param = -np.tanh(2*(param_ang_error-np.pi/2))*np.tanh(2*(param_ang_error+np.pi/2))
        dist_error = param*np.linalg.norm(error)

        ###############
        quat = R.from_quat(bot_quat)
        heading_angle = (quat.as_euler('zyx',degrees=False)[-1])
        if heading_angle<0:
            heading_angle = -(heading_angle+np.pi)
        else:
            heading_angle = -(heading_angle - np.pi)
        ################

        ang_error = np.arctan2((error[0]/np.abs(error[0]))*error[1],error[0]) - heading_angle
        # print(np.rad2deg(ang_error), ang_error,dist_error)

        
        
        # print(np.rad2deg(heading_angle))
        if len(self.error_buffer)>10:
            self.error_buffer.pop(0)
        self.error_buffer.append([dist_error,ang_error])
        prop_x = self.K_x*dist_error
        prop_y = self.K_y*ang_error

        # print(dist_error,ang_error)

        intgr_x = self.K_I_x*self.sum_errors(type='x')
        intgr_y = self.K_I_y*self.sum_errors(type='y')

        try:
            
            diff_x = self.K_D_x*(self.error_buffer[-1][0] - self.error_buffer[-2][0])
            diff_y = self.K_D_y*(self.error_buffer[-1][1] - self.error_buffer[-2][1])
        except:
            diff_x = self.K_D_x*(self.error_buffer[-1][0] - 0)
            diff_y = self.K_D_y*(self.error_buffer[-1][1] - 0)

        v_x = prop_x + intgr_x + diff_x - self.K_damp_x*self.v_x
        v_y = prop_y + intgr_y + diff_y - self.K_damp_y*self.v_y
        self.v_x = v_x
        self.v_y = v_y

        return v_x, v_y
