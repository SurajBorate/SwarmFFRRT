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
    #initformation = np.array([[-100,-100,1],[100,100,1],[100,-100,1],[-100,100,1]]).T 
    initFormation = np.array([[-180,-180,1],[180,180,1],[180,-180,1],[-180,180,1]]).T
    waypoints = np.load('4robot1.npy')

    L=np.shape(waypoints)[0]

    locs = np.array([[[1,2],[1,2],[1,2],[1,2]]])
    for i in range(L):
        location = getPosesFromFormation(waypoints[i][:],initFormation)
        if i !=0:    
            locs=np.vstack((locs,np.array([location])))
        else:
            locs[i] = location
    locs1 = locs[::-1]
    locs1 = locs1 - locs1[0]
    colorlist = ['b','r','g','k']
    # for i in range(np.shape(locs)[0]-1):
    #     plt.plot([locs1[i,0,0],locs1[i+1,0,0]],[locs1[i,0,1],locs1[i+1,0,1]],'r')
    # for i in range(np.shape(locs)[0]-1):
    #     plt.plot([locs1[i,1,0],locs1[i+1,1,0]],[locs1[i,1,1],locs1[i+1,1,1]],'b')
    for j in range(np.shape(locs1)[1]):
        for i in range(np.shape(locs1)[0]-1):
            plt.plot([locs[i,j,0],locs[i+1,j,0]],[locs[i,j,1],locs[i+1,j,1]],colorlist[j])
    plt.show()
    plt.figure
    
    for j in range(np.shape(locs1)[1]):
        for i in range(np.shape(locs1)[0]-1):
            plt.plot([locs1[i,j,0],locs1[i+1,j,0]],[locs1[i,j,1],locs1[i+1,j,1]],colorlist[j])
    # for i in range(np.shape(locs)[0]-1):
    #     plt.plot([locs[i,1,0],locs[i+1,1,0]],[locs[i,1,1],locs[i+1,1,1]],'b')
    plt.show()
    print(np.shape(locs1))
    print(locs1)
    np.save('locs1.npy',locs1)
    