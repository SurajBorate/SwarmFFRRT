import numpy as np
from itertools import combinations
from matplotlib import pyplot as plt
np.random.seed(2)

def lineintersect(l1,l2): # gives intersection point of two lines
    a1 = l1[0]
    b1 = l1[1]
    c1 = l1[2]
    a2 = l2[0]
    b2 = l2[1]
    c2 = l2[2]
    if (a1 * b2 - a2 * b1) != 0:
        p0 = [round((b1*c2-b2*c1)/(a1*b2-a2*b1),4),round((c1*a2-c2*a1)/(a1*b2-a2*b1),4)]
    else:
        p0 = [-111,-111]
    return p0

def satisfyconstraint(l1,p0): # substitutes point in constraint equation and return bool
    x0 = p0[0]
    y0 = p0[1]
    satisfy = 0
    # substitute the point in constraint
    if l1[0]*p0[0]+l1[1]*p0[1] >= -l1[2]*0.95:
        satisfy = 1
    return satisfy

def linefrompoint(p0,p1): # creates line between points
    x0 = p0[0]
    y0 = p0[1]
    x1 = p1[0]
    y1 = p1[1]
    a1 = y0-y1
    b1 = x1-x0
    c1 = y0*x1-y0*x0-y1*x0
    l1 = [a1, b1, c1]
    return l1

def plot_lines(Lines1):
    tt = np.linspace(0,5,100)
    for i in Lines1:
        a = i[0]
        b = i[1]
        c = i[2]
        if b!=0:
            y = (-c-a*tt)/b
            plt.plot(tt,y,'-')
        if b == 0:
            x = [-c/a,-c/a]
            y = [0, 5]
            plt.plot(x,y)

def find_Scaling_Poly(initformation,rad,Sxmax2,Symax2,Plotty=False):
    n  = len(initformation.T) # no of agents
    radius = rad*np.ones((1, n)) # radius array of size 1xn
    radius[0][3] = 250
    v = list(range(1,n+1)) # list of integers from 1 to n, these are agent-numbers
    point_index = combinations(v,2) # all possible pairs of 2's

    lines = [] # empty line array

    for i in list(point_index):
        p0 = initformation[:,i[0]-1] # retrieve first point
        p1 = initformation[:,i[1]-1] # retrieve second point
        delp = p1-p0
        delx2 = (delp[0])**2
        dely2 = (delp[1])**2
        c2 = (radius[0][i[0]-1]+radius[0][i[1]-1])**2
        l1 = [delx2, dely2, -c2]
        lines.append(l1)
    lines.append([1,0,0]) # append x axis to line list
    lines.append([0,1,0]) # append y axis to line list
    Lines1 = lines.copy()
    Lines1.append([1,0,-Sxmax2]) # all lines
    Lines1.append([0,1,-Symax2]) # all lines
    nc2=np.math.factorial(n)/(np.math.factorial(n-2)*2)
    z = list(range(1,int(nc2)+2+1+2))
    line_index = combinations(z,2)
    points = []
    for i in line_index:
        p0 = lineintersect(Lines1[i[0]-1],Lines1[i[1]-1])
        if p0 != [-111,-111]:
            points.append(p0)
    #all intersection points

    # list of critical points satisfying all constraints
    # all lines
    

    LINES = np.array(lines)
    LINES=np.around(LINES,2)
    lines = list(LINES)

    
    POINTS = np.array(points)
    POINTS=np.around(POINTS,2)
    points = list(POINTS)
    s = 0
    sum = 0
    cp = []

    for p in points:
        for l in lines:
            s = satisfyconstraint(l,p)
            sum = sum+s

        if sum == len(lines):

            cp.append(p)

        sum = 0


    critical_points = np.unique(cp,axis=0)
    
    if Plotty == True:
        plot_lines(lines)
        for p in critical_points:
            plt.plot(p[0],p[1],'b*')
        plt.xlabel("$(\sigma_x)^2$")
        plt.ylabel("$(\sigma_y)^2$")
        plt.xlim(0,5)
        plt.ylim(0,5 )
        plt.show()
    return critical_points

if __name__ == "__main__":
    Sxmax2 = 4
    Symax2 = 4
    rad = 0.2
    initformation = np.array([[0,1,1],[0.6,0.75,1],[0.6,-0.75,1],[0,-1,1],[-0.6,-0.75,1],[-0.6,0.75,1]]).T
    print(find_Scaling_Poly(initformation,rad,Sxmax2,Symax2,True))
   