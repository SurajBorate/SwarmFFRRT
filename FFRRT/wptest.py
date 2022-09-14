import numpy as np
from matplotlib import pyplot as plt
def getPosesFromFormation(node,initFormation):
    
    xc, yc, theta, sx, sy = node
    theta = theta 
    matr = np.array(
        [
            [sx * np.cos(theta), -sx * np.sin(theta), xc],
            [sy * np.sin(theta), sy * np.cos(theta), yc],
            [0, 0, 1]
        ]
    )

    locs = np.dot(matr, initFormation)
    locs = locs[:2].T.astype(np.int)
    return locs
if __name__ == "__main__":
    initFormation = np.array([[-200,-200,1],[200,200,1],[200,-200,1],[-200,200,1]]).T 
    #initFormation = np.array([[0,-250,1],[0,250,1]]).T
    # waypoints = np.load('rrtstar_fourbots_wpa.npy')
    waypoints = np.load('waypoint_arraynew.npy')
    #print(waypoints)
    L=np.shape(waypoints)[0]
    #print(L)
    locs = np.array([[[1,2],[1,2],[1,2],[1,2]]])
    for i in range(L):
        location = getPosesFromFormation(waypoints[i][:],initFormation)
        if i !=0:    
            locs=np.vstack((locs,np.array([location])))
        else:
            locs[i] = location
    locs1 = locs[::-1]
    locs1 = locs1 - locs1[0]
    
    for i in range(np.shape(locs)[0]-1):
        plt.plot([locs[i,0,0],locs[i+1,0,0]],[locs[i,0,1],locs[i+1,0,1]],'r')
    for i in range(np.shape(locs)[0]-1):
        plt.plot([locs[i,1,0],locs[i+1,1,0]],[locs[i,1,1],locs[i+1,1,1]],'b')
    for i in range(np.shape(locs)[0]-1):
        plt.plot([locs[i,2,0],locs[i+1,2,0]],[locs[i,2,1],locs[i+1,2,1]],'g')
    for i in range(np.shape(locs)[0]-1):
        plt.plot([locs[i,3,0],locs[i+1,3,0]],[locs[i,3,1],locs[i+1,3,1]],'y')
    #print(locs[1,0,1])
    # start point
    print(locs[-1])
    # start in camera
    print("x")
    print(locs[-1,:,0]/3)
    print("y")
    print(locs[-1,:,1]/2)
    plt.show()
    plt.figure
    for i in range(np.shape(locs1)[0]-1):
        plt.plot([locs1[i,0,0],locs1[i+1,0,0]],[locs1[i,0,1],locs1[i+1,0,1]],'r')
    for i in range(np.shape(locs)[0]-1):
        plt.plot([locs1[i,1,0],locs1[i+1,1,0]],[locs1[i,1,1],locs1[i+1,1,1]],'b')
    for i in range(np.shape(locs)[0]-1):
        plt.plot([locs1[i,2,0],locs1[i+1,2,0]],[locs1[i,2,1],locs1[i+1,2,1]],'g')
    for i in range(np.shape(locs)[0]-1):
        plt.plot([locs1[i,3,0],locs1[i+1,3,0]],[locs1[i,3,1],locs1[i+1,3,1]],'y')
    plt.show()
    np.save('locs1_4robot_ffrrtstar_N3.npy',locs1)
    np.save('locs0_4robot_ffrrtstar_N3.npy',locs)