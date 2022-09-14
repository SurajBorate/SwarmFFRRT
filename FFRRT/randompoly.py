import numpy as np
from itertools import combinations
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
import random
from LPsigma import find_Scaling_Poly
random.seed(2)
np.random.seed(2)
def AOT(pts):
    x1 = pts[0][0]
    y1 = pts[0][1]
    x2 = pts[1][0]
    y2 = pts[1][1]
    x3 = pts[2][0]
    y3 = pts[2][1]
    B = np.array([[(x1 - x2),(x1 - x3)],
    [(y1 - y2),(y1 - y3)]])
    return 0.5 * np.linalg.det(B)
def plot_poly(y):
    p = Polygon(y, facecolor='k')
    fig, ax = plt.subplots()
    ax.add_patch(p)
    ax.set_xlim([0, 10])
    ax.set_ylim([0, 10])

def sample_in_poly_plot(poly1,N):
    npo = len(poly1)

    vecs = poly1[1:npo,:]-poly1[0,:] # all vecs with first vertex
    Areas = np.zeros([len(poly1)-2,1])
    for i in range(len(poly1)-2):
        B = np.array([poly1[0],poly1[i+1],poly1[i+2]])
        Areas[i] = AOT(B)

    NAreas = Areas/(np.sum(Areas))
    NAreas = NAreas.T
    pn = 0
    i = 1
    samples = np.zeros([400,2])
    plot_poly(poly1)
    for ii in range(N):
        count_list = [i+1 for i in range(npo-2)]
        ano = random.choices(count_list,NAreas[0]*100,k = 1)
        pvec = poly1[0]+vecs[ano[0]-1]*np.random.rand(1,1)+vecs[ano[0]]*np.random.rand(1,1)
        x3 = pvec[0][0]
        y3 = pvec[0][1]
        x1 = poly1[0][0] + vecs[ano[0]-1][0]
        y1 = poly1[0][1] + vecs[ano[0]-1][1]
        x2 = poly1[0][0] + vecs[ano[0]][0]
        y2 = poly1[0][1] + vecs[ano[0]][1]
        a = y2-y1
        b = -(x2-x1)
        c = y1*(x2-x1)-x1*(y2-y1)
        if a * x3 + b * y3 + c > 0 :
            xc = (x1 + x2) / 2
            yc = (y1 + y2) / 2
            x4 = 2 * xc - x3
            y4 = 2 * yc - y3
            plt.plot(x4, y4, 'b*')
        else :
            x4 = x3
            y4 = y3
            plt.plot(x4, y4, 'b*')
    plt.show()

def sample_in_poly(poly1):
    npo = len(poly1)
    vecs = poly1[1:npo,:]-poly1[0,:] # all vecs with first vertex
    Areas = np.zeros([len(poly1)-2,1])

    for i in range(len(poly1)-2):
        B = np.array([poly1[0],poly1[i+1],poly1[i+2]])
        Areas[i] = AOT(B)

    NAreas = Areas/(np.sum(Areas))
    NAreas = NAreas.T
    pn = 0
    i = 1
    samples = np.zeros([400,2])        
    count_list = [i+1 for i in range(npo-2)]
    ano = random.choices(count_list,NAreas[0]*100,k = 1)
    pvec = poly1[0]+vecs[ano[0]-1]*np.random.rand(1,1)+vecs[ano[0]]*np.random.rand(1,1)
    x3 = pvec[0][0]
    y3 = pvec[0][1]
    x1 = poly1[0][0] + vecs[ano[0]-1][0]
    y1 = poly1[0][1] + vecs[ano[0]-1][1]
    x2 = poly1[0][0] + vecs[ano[0]][0]
    y2 = poly1[0][1] + vecs[ano[0]][1]
    a = y2-y1
    b = -(x2-x1)
    c = y1*(x2-x1)-x1*(y2-y1)
    if a * x3 + b * y3 + c > 0 :
        xc = (x1 + x2) / 2
        yc = (y1 + y2) / 2
        x4 = 2 * xc - x3
        y4 = 2 * yc - y3
        
    else :
        x4 = x3
        y4 = y3
            
    return np.array([x4,y4])

def sort_points_clock(poly1):
    centroid = np.sum((poly1),axis=0)/(len(poly1))
    vecy =  poly1[0] - centroid
    dict1 = {}
    sortpoly=np.zeros([len(poly1),2])
    for pol in poly1:
        vec =  pol - centroid
       # D = np.dot(vec,vecy)/(np.linalg.norm(vec)*np.linalg.norm(vecy))
        D = np.arctan2(vec[1],vec[0])
        dict1.update([(D,pol)])
    i = 0
    dictionary1 = sorted(dict1.items())
    sorteddict = dict(dictionary1)
    for key in sorteddict:
        sortpoly[i] = sorteddict[key]
        i = i+1
    return sortpoly
    
def generate_random_scaling(initformation,rad,Sxmax2,Symax2):
    poly1 = find_Scaling_Poly(initformation,rad,Sxmax2,Symax2,False) 
    sortpoly = sort_points_clock(poly1)
    S = sample_in_poly(sortpoly)
    return S

if __name__ == "__main__":
    Sxmax2 = 4
    Symax2 = 4
    initformation = np.array([[0,1,1],[0.6,0.75,1],[0.6,-0.75,1],[0,-1,1],[-0.6,-0.75,1],[-0.6,0.75,1]]).T
    rad = 0.2# radius array of size 1xn
    poly1 = find_Scaling_Poly(initformation,rad,Sxmax2,Symax2,False) 
    sortpoly = sort_points_clock(poly1)
    #plot_poly(sortpoly)
    #plt.show()
    N = 50
    #sample_in_poly_plot(sortpoly,N)
    print(sample_in_poly(sortpoly))
    print(generate_random_scaling(initformation,rad,Sxmax2,Symax2))


    