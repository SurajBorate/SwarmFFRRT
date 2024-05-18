import numpy as np
#from scipy.linalg import pascal
from matplotlib import pyplot as plt
import cv2

def cubic(x0,x1,v0,v1,t0,tf):
    #print('t0'+str(t0))
    A = np.array([[1, t0, t0**2, t0**3],[0, 1, 2*t0, 3*(t0**2)],[1, tf, tf**2, (tf**3)],[0, 1, 2*tf, 3*(tf**2)]])
    #A = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],dtype='float')
    B = np.array(A,dtype='float')
    b = np.array([[x0],[v0],[x1],[v1]],dtype='float')    
    Ainv = np.linalg.inv(A)
    p = np.matmul(Ainv,b)
    return p

def find_cubic_traj(waypoints,t0,tf,V,dt):
    n = np.shape(waypoints)
    n = n[0]
    #print(n)
    x = waypoints[:,0]
    #print(x)
    y = waypoints[:,1]
    delx = np.diff(x)
    dely = np.diff(y)
    # print(delx)
    # print(dely)
    # plt.plot(x,y,'o')
    # plt.show()
    O = np.zeros([n-1,1])
    D = np.zeros([n-1,1])
    delT = np.zeros([n-1,1])
    #print(delT)
    T = np.zeros([n-1,1])
    X0 = np.zeros([n-1,1])
    Xf = np.zeros([n-1,1])
    Y0 = np.zeros([n-1,1])
    Yf = np.zeros([n-1,1])
    Vx0 = np.zeros([n-1,1])
    Vxf = np.zeros([n-1,1])
    Px = np.zeros([n-1,4])
    Py = np.zeros([n-1,4])
    for i in range(0,n-1):
        o=np.arctan2(dely[i],delx[i])
        O[i] = o
        D[i] = delx[i]**2+dely[i]**2
    
    for i in range(0,n-1):
        delT[i] = (tf-t0)*(D[i]/np.sum(D))
        T[i] = t0+np.sum(delT[0:i+1])
        
    T = np.vstack(([0],T))
   
    Vx = V*np.cos(O)
    Vx0 = Vx
    Vxf = np.vstack((Vx[1:n-1],[0]))
    Vy = V*np.sin(O)
    Vy0 = Vy
    Vyf = np.vstack((Vy[1:n-1],[0]))
    T0 = T
    Tf = np.vstack((T[1:n-1],[tf]))
    
    X0 = x[0:n-1]
    Xf = x[1:n]
    Y0 = y[0:n-1]
    Yf = y[1:n]
    
    #print(np.shape(X0))
    for i in range(0,n-1):
         px = cubic(X0[i],Xf[i],Vx0[i],Vxf[i],T0[i][0],Tf[i][0])
         Px[i,0:4] = np.transpose(px)
         py = cubic(Y0[i],Yf[i],Vy0[i],Vyf[i],T0[i][0],Tf[i][0])
         Py[i,0:4] = np.transpose(py)
    #print(Py)
    for i in range(0,n-1):
          t=np.linspace(T0[i][0],Tf[i][0],int((Tf[i][0]-T0[i][0])/dt))
          Xtraj = Px[i][0]+Px[i][1]*t+Px[i][2]*t**2+Px[i][3]*t**3
          Ytraj = Py[i][0]+Py[i][1]*t+Py[i][2]*t**2+Py[i][3]*t**3
          #plt2 = plt.plot(Xtraj,Ytraj)
          
        #   VXtraj = Px[i][1]+2*Px[i][2]*t+3*Px[i][3]*t**2
        #   VYtraj = Py[i][1]+2*Py[i][2]*t+3*Py[i][3]*t**2
        #   AXtraj = 2*Px[i][2]*t+6*Px[i][3]*t
        #   AYtraj = 2*Py[i][2]*t+6*Py[i][3]*t
        #   theta = np.arctan2(VYtraj,VXtraj)
        #   thetadot = (1/(VXtraj**2+VYtraj**2))*((VXtraj*AYtraj-VYtraj*AXtraj))
        #   V = np.sqrt(np.square(VXtraj)+np.square(VYtraj))
        #   plt.figure(1)
        #   plt1 = plt.plot(t,thetadot)
        #   plt.figure(2)
           
        #   plt.figure(3)
        #   plt3 = plt.plot(t,Xtraj)      
    #plt.show()
    
    #print(Px,Py,Tf)
    return(Px,Py,Tf)

# def inverse_differential_kinematics(Xtraj,Ytraj,VXtraj,VYtraj,theta):
#     theta = np.arctan2(VXtraj,VYtraj)
#     thetadot = 
#     n = np.shape(theta)
#     n = n[0]
#     for i in range(n):
#         twist=R*np.array([[Vx[i][0]],[Vy[i][0]]])

def find_xy_cubic(Px,Py,t):
    x = Px[0]+Px[1]*t+Px[2]*t**2+Px[3]*t**3
    y = Py[0]+Py[1]*t+Py[2]*t**2+Py[3]*t**3
    return x,y


if __name__=='__main__':
    #waypoints = np.array([[0,0],[1,0],[1,1],[0,1]])
    ways = np.load('way.npy')
    print(ways)
    wnn = np.shape(ways)
    wnn = wnn[0]
    waypoints_r1 = np.zeros((wnn,2))
    waypoints_r2 = np.zeros((wnn,2))
    for i in range(wnn):
        waypoints_r1[i][:] = ways[i][0]
        waypoints_r2[i][:] = ways[i][1]
    print(waypoints_r1)
    Px,Py,Tf = find_cubic_traj(waypoints_r1, 0, 60, 0.1, 0.001)
    print(Px)
    
	
