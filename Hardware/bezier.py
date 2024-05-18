
import numpy as np
from scipy.linalg import pascal


def find_bezier_trajectory(coordinates, points,t):
    n=len(coordinates)

    pascal_coord=pascal(n,kind='lower')[-1]
    

    bezier_x=np.zeros(points)
    bezier_y=np.zeros(points)

    for i in range(n):
        k=(t**(n-1-i))
        l=(1-t)**i
        bezier_x+=np.multiply(l,k)*pascal_coord[i]*coordinates[n-1-i][0]
        bezier_y+=np.multiply(l,k)*pascal_coord[i]*coordinates[n-1-i][1]
    
    bezier_xd=[]
    bezier_yd=[]
    for i in range(len(bezier_x)):
        bezier_xd.append(np.round(bezier_x[i],3))
        bezier_yd.append(np.round(bezier_y[i],3))

    bezier_coordinates = np.transpose([bezier_xd, bezier_yd])
    # print(bezier_coordinates)
    return bezier_coordinates

if __name__=='__main__':
    points = [[0,0],[1,1],[2,2],[3,3]]
    bez = find_bezier_trajectory(points, 10, 1)
    print(bez)