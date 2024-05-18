import numpy as np
from scipy.spatial.transform import Rotation as R

class PID_Controller():
    def __init__(self,):
        self.K_th = 0.5
        self.K_I_th = 0
        self.K_D_th = 0
        self.K_damp_th = 0
        
        self.K_st = 0.5
        self.K_I_st = 0.0 # delt will be adjusted in K_i value
        self.K_D_st = 0
        self.K_damp_st = 0.0

        self.v_th = 0
        self.v_st = 0

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
    def track(self,error, bot_quat):
        
        quat = R.from_quat(bot_quat)
        heading_angle = (quat.as_euler('zyx',degrees=False)[-1])
        if heading_angle<0:
            heading_angle = -(heading_angle+np.pi)
        else:
            heading_angle = -(heading_angle - np.pi)

        # print('heading_angle : ',heading_angle)
        rot_matrix = np.array([[ np.cos(heading_angle),np.sin(heading_angle)],
                               [-np.sin(heading_angle),np.cos(heading_angle)]])
        
        local_error = np.matmul(rot_matrix, np.array(error).T)


        param_ang_error = np.arctan2(error[1],error[0]) - heading_angle  
        
        param = -np.tanh(4*(param_ang_error-np.pi/2))*np.tanh(4*(param_ang_error+np.pi/2))
        # # param_ang_error = np.arctan2(error[1],error[0]) - heading_angle  
        # param = np.tanh(np.cos(heading_angle)*error[0] + error[1]*np.sin(heading_angle))
        # param = np.tanh()
        dist_error = param*np.linalg.norm(error) 

        # print(param, error, dist_error)
        ###############
        # Will be used in Turtlebot to convert quaternions to required heading angle

        ################

        ang_error = np.arctan2(local_error[1],local_error[0])
        if ang_error>np.pi/2:
            ang_error = ang_error - np.pi
        elif ang_error<-np.pi/2:
            ang_error = ang_error + np.pi

        if len(self.error_buffer)>10:
            self.error_buffer.pop(0)
        self.error_buffer.append([dist_error,ang_error])
        prop_x = self.K_th*dist_error
        prop_y = self.K_st*ang_error

        # print(dist_error,ang_error)

        intgr_th = self.K_I_th*self.sum_errors(type='x')
        intgr_st = self.K_I_st*self.sum_errors(type='y')

        # print(prop_x)
        try:
            
            diff_x = self.K_D_th*(self.error_buffer[-1][0] - self.error_buffer[-2][0])
            diff_y = self.K_D_st*(self.error_buffer[-1][1] - self.error_buffer[-2][1])
            
        except:
            diff_x = self.K_D_th*(self.error_buffer[-1][0] - 0)
            diff_y = self.K_D_st*(self.error_buffer[-1][1] - 0)

        v_th = prop_x + intgr_th + diff_x - self.K_damp_th*self.v_th
        v_st = prop_y + intgr_st + diff_y - self.K_damp_st*self.v_st # omega 
        self.v_th = v_th
        self.v_st = v_st

        # print(v_th, v_st)

        # return v_th,v_st, {'dist_error':dist_error,'ang_error':np.rad2deg(ang_error)}
        return v_th,v_st