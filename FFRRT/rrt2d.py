from os import close
import numpy as np
import cv2
from tomlkit import key
from tqdm import tqdm
from randompoly import *
from LPsigma import *
np.random.seed(2)
weights = np.array([0.47, 0.47, 0.3, 0, 0])
weights /= np.sum(weights)
initFormation = np.array([
        [0, 0, 1], 
        [0, 0, 1], 
        [0, 0, 1], 
        [0, 0, 1]
    ]).T
initFormation = np.array([[-200,-200,1],[200,200,1],[200,-200,1],[-200,200,1]]).T  # for 4 robots

dt = 0.01
width = 3000
height = 2000
maxrobotrad = 160
markersqbytwo = 96
obssize = 500

obstacleMap = np.zeros((height, width), np.uint8)
# center block for Split Merge case
#obstacleMap[ int(height/2)-int(obssize/2)-maxrobotrad:int(height/2)+int(obssize/2)+maxrobotrad,int(width/2)-int(obssize/2)-maxrobotrad:int(width/2)+int(obssize/2)+maxrobotrad, ] = 255

# two obs
obstacleMap[0:189*2+int(obssize/2)+maxrobotrad,632*3-int(obssize/2)-maxrobotrad:632*3+int(obssize/2)+maxrobotrad, ] = 255
obstacleMap[2000-189*2-int(obssize/2)-maxrobotrad:2000,632*3-int(obssize/2)-maxrobotrad:632*3+int(obssize/2)+maxrobotrad, ] = 255
# markers
obstacleMap[ 0:markersqbytwo+maxrobotrad,0:markersqbytwo+maxrobotrad, ] = 255
obstacleMap[ height-(markersqbytwo+maxrobotrad):height,0:markersqbytwo+maxrobotrad, ] = 255
obstacleMap[ 0:markersqbytwo+maxrobotrad,width-(markersqbytwo+maxrobotrad):width, ] = 255
obstacleMap[ height-(markersqbytwo+maxrobotrad):height,width-(markersqbytwo+maxrobotrad):width, ] = 255
# arena border
obstacleMap[ 0:height,0:maxrobotrad, ] = 255
obstacleMap[ 0:height,width-maxrobotrad:width, ] = 255
obstacleMap[ 0:maxrobotrad,0:width, ] = 255
obstacleMap[ height-maxrobotrad:height,0:width, ] = 255
class Node():
    def __init__(self, state = [0, 0, 0, 0, 0], distWeights = np.array([0.47, 0.47, 0.02, 0.02, 0.02]), id = 0):
        self.state = np.array(state)
        self.parent : Node = None
        self.distWeights = distWeights
        self.cost = 0
        self.id = id

    def connectChild(self, node):
        node.parent = self

    def draw(self, img, color):
        cv2.circle(img, (int(self.state[0]), int(self.state[1])), 3, color, -1)
        return img

    def norm(self):
        return np.dot(self.distWeights, self.state)

    def __add__(self, node):
        if type(node) == Node:
            return Node(self.state + node.state)
        else:
            return self.state + node

    def __mul__(self, node):
        return Node(self.state * node)

    def __sub__(self, node):
        if type(node) == Node:
            return Node(self.state - node.state)
        else:
            return self.state - node

    def __eq__(self, a):
        if np.isclose(self.state, a.state).all():
            return True
        return False

    def __hash__(self):
        return id(self)

    def __str__(self):
        return str(self.state)


def angDist(ang1, ang2):
    maxi = max(ang1, ang2)
    mini = min(ang1, ang2)
   
    diff = maxi - mini
    
    if diff <=np.pi:
        dist_angle = diff
    
    if diff > np.pi:
        dist_angle = np.pi - diff

    return dist_angle

def distance(nodeA, nodeB, weights = weights, useMap = False, newDist = True, distanceScaler = 1):


    if not newDist:
        ang1 = nodeA.state[2]
        ang2 = nodeB.state[2]
        
        dist = (nodeA - nodeB)
        dist = dist.state / (dist.norm() + 1e-6)
        
        dist[2] = angDist(ang1, ang2)/(2*np.pi)
        ret = np.sqrt(np.dot((nodeA.state - nodeB.state)**2, weights))
        return ret 

    else:
        # print('Distance')
        xi = initFormation[0]
        yi = initFormation[1]
        Nx = np.sum(np.dot(xi, xi.T))
        Ny = np.sum(np.dot(yi, yi.T))

        Nxy = np.sum(np.dot(xi, yi.T))

        xca, yca, sxa, sya, thetaA= nodeA.state
        xcb, ycb, sxb, syb, thetaB= nodeB.state
        theta21 = thetaB - thetaA
        n = len(initFormation[0])
        a = Nx * (sxa ** 2 - 2*sxa * sxb *np.cos(theta21) + sxb**2)
        b = Ny * (sya ** 2 - 2*sya * syb *np.cos(theta21) + syb**2)
        c = 2* Nxy *(sya *sxb  - syb * sxa) * np.sin(theta21)
        d = n * ((xca - xcb) ** 2 + (yca - ycb)**2)
        J = np.sqrt(a + b - c + d)   
        return J * distanceScaler
        
            

    
def findCloseNodes(node, nodes, weights, thresh = float('inf'),  newDist = False):
    nodes = list(nodes)
    distances = []
    finalnodes = []
    for i in range(len(nodes)):
        dist = distance(node, nodes[i], weights,  newDist = newDist)
        if dist < thresh:
            distances.append(dist)
            finalnodes.append(nodes[i])
    
    sortedNodes = sorted(list(zip(distances, finalnodes)), key=lambda x: x[0])
    distances = [x[0] for x in sortedNodes]
    finalNodes = [x[1] for x in sortedNodes]
    return np.array(distances), finalNodes


def adjustDirection(nodeToExtend, directionNode, obstacleMap):
    global weights
    cx, cy = nodeToExtend.state[0:2].astype(int)
    sx = nodeToExtend.state[3]
    sy = nodeToExtend.state[4]

    # hwin = int( np.sqrt((sx * 50)** 2 + (sy * 50) ** 2))
    poses = np.array(getPosesFromFormation(nodeToExtend))
    win = 0
    xmin = max(np.min(poses[:, 0]) + win, 0)
    xmax = min(np.max(poses[:, 0]) - win, obstacleMap.shape[1])
    ymin = max(np.min(poses[:, 1]) + win, 0)
    ymax = min(np.max(poses[:, 1]) - win, obstacleMap.shape[0])
    window = obstacleMap[ymin:ymax, xmin:xmax]
    sampledImage[ymin:ymax, xmin:xmax] = 40

    return directionNode


def getNodeExtension(node, directionNode, eps):
    delta = (directionNode - node)
    val = distance(directionNode, node,weights=np.array([1, 1, 1, 1,1]), useMap= True,newDist=False)

    if val >= eps:
        extendedNode = node + delta * (eps/val); 
    else:
        extendedNode = directionNode
    
    return extendedNode

it = 0

def render(visitedNodes, goal, start, nodeToExtend, extendedNode, directionNode, obstacleMap):
    global it, goalRadius
    it += 1
    # print('Iteration', it)
    
    img = cv2.cvtColor(obstacleMap.copy(), cv2.COLOR_GRAY2BGR)
    img[obstacleMap == 255] = 255
    cv2.circle(img, (int(goal.state[0]), int(goal.state[1])), 100, (0, 0, 128), -1)
    cv2.circle(img, (int(start.state[0]), int(start.state[1])), 100, (0, 128, 0), -1)

    color = (0, 255, 0)

    for node in visitedNodes:
        img = node.draw(img, (0, 0, 255))
        parent = node.parent
        if parent is not None:
            cv2.line(img, (int(node.state[0]), int(node.state[1])), (int(parent.state[0]), int(parent.state[1])), (0, 0, 200), 1)

    
    img = goal.draw(img, (0, 255, 0))
    img = start.draw(img, (255, 0, 0))
    img = directionNode.draw(img, (255, 0, 255))
    img = nodeToExtend.draw(img, (255, 255, 0))
    img = extendedNode.draw(img, (255, 255, 255))
    drawFormation(extendedNode, img, color = (0, 255, 0), verbose = False)
    drawFormation(start, img, color = (0, 255, 255))

    cv2.line(img, (int(extendedNode.state[0]), int(extendedNode.state[1])), (int(nodeToExtend.state[0]), int(nodeToExtend.state[1])), (0, 0, 200), 1)
    
    return img

def drawFormation(node, img, color = (255, 255, 255), verbose = False, prevNode = None):
    global initFormation
    if prevNode is not None:
        newLocs = getPosesFromFormation(prevNode)

    locs = getPosesFromFormation(node)
    colors = [(255, 0, 0), (0, 255, 0), (0, 255, 255), (255, 255, 0)]
    for i, loc in enumerate(locs):
        cv2.circle(img, (int(loc[0]), int(loc[1])), 10, colors[i], -1)
        if prevNode is not None:
            cv2.line(img, (int(loc[0]), int(loc[1])), (int(newLocs[i][0]), int(newLocs[i][1])), colors[i], 1)
            cv2.circle(img, (int(newLocs[i][0]), int(newLocs[i][1])), 10, colors[i], -1)
    
    
    #if verbose:
        #print(locs)
        #print(node.state)

    return img

    

def drawBackTrack(node, img, goal, start, returnPath = False):

    color = (0, 255, 0)
    if goal is not None:
        cv2.circle(img, (int(goal.state[0]), int(goal.state[1])), 50, (128, 128, 0), -1)
        img = goal.draw(img, (0, 255, 0))
    if start is not None:
        cv2.circle(img, (int(start.state[0]), int(start.state[1])), 50, (128, 128, 0), -1)
        img = start.draw(img, (0, 255, 0))

    keypoints = []
    while node is not None:
        img = node.draw(img, (255, 255, 0))
        img = (img * 1).astype(np.uint8)
        keypoints.append(node.state)
        parent = node.parent
        drawFormation(node, img, color = (0, 255, 0), prevNode = parent)
        if parent is not None:
            cv2.line(img, (int(node.state[0]), int(node.state[1])), (int(parent.state[0]), int(parent.state[1])), (0, 0, 200), 1)
        
        node = parent
    show(img, winname='a', waitkey=1)
    if returnPath:
        return img, keypoints
    return img

def show(img, winname , waitkey = 0):

    cv2.imshow(winname, img)

    if cv2.waitKey(waitkey) == ord('q'):
        exit()

def getPosesFromFormation(node):
    global initFormation
    xc, yc, theta, sx, sy = node.state 
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


def isColliding(extendedNode, obstacleMap, nodeToExtend=None , steps = 1):
    # x, y = int(node.state[0]), int(node.state[1])
    if nodeToExtend is None:
        statesToCheck = [extendedNode]
        
    else:
        
        statesToCheck = [nodeToExtend + (nodeToExtend - extendedNode) * (i / steps) for i in range(steps + 1)]
        # Todo convert nodeToExtend and extended node into locs and interpolate between locs
    for state in statesToCheck[::-1]:

        locs = getPosesFromFormation(state)
        for loc in locs:
            try:
                if obstacleMap[loc[1], loc[0]] == 255 :
                    return True
                if loc[0] <= 0 or loc[0] >= 3000 or loc[1] <= 0 or loc[1] >= 2000:
                    return True
            except:
                continue
    
    return False

def isCollidingnew(extendedNode, obstacleMap, nodeToExtend=None , steps = 1):
    # x, y = int(node.state[0]), int(node.state[1])   
     
    if nodeToExtend is None:
        statesToCheck = [extendedNode]
        locs = getPosesFromFormation(extendedNode)
        
    else:
        locs1 = getPosesFromFormation(extendedNode)
        locs2 = getPosesFromFormation(nodeToExtend)
        
        # Todo convert nodeToExtend and extended node into locs and interpolate between locs

    # for loc in locs:
    #         try:
    #             if obstacleMap[loc[1], loc[0]] == 255 :
    #                 return True
    #             if loc[0] <= 0 or loc[0] >= 3000 or loc[1] <= 0 or loc[1] >= 2000:
    #                 return True
    #         except:
    #             continue
    
    return False


def isColliding3(Node, obstacleMap):
    # x, y = int(node.state[0]), int(node.state[1])
    locs = getPosesFromFormation(Node)
    for loc in locs:
        try:
            if obstacleMap[loc[1], loc[0]] == 255 :
                return True
            if loc[0] <= 0 or loc[0] >= 3000 or loc[1] <= 0 or loc[1] >= 2000:
                return True
        except:
            continue
    
    return False

def RRT(dic):
    global initFormation
    global visitedNodes, sampledImage
    global goalRadius
    numNodes = 2000
    obstacleMap = dic['obstacleMap']    
    eps = dic['maxDistanceExtend']
    goalRadius = dic['goalRadius']
    neighbourRadius = dic['neighbourRadius'] # RRT star
    nAgents = dic['nAgents']
    radius = dic['radius']
    goal = dic['goal']
    heur = dic['heur']
    start = dic['start']
    newDist = dic['newDist']
    min_state = dic['min_state']
    max_state =dic['max_state']
    sp = dic['sortpoly']
    sampledImage = obstacleMap.copy()
    sampledImage[sampledImage == 0] = 40
    edgeCollStep = dic['edgeCollStep']
    
    #initFormation = np.array([
    #    [radius * np.cos(theta+ np.pi ), radius * np.sin(theta + np.pi), 1] for theta in np.arange(0, 2*np.pi, 2*np.pi/nAgents)
    
    #initFormation = np.array([[-100,-100,1],[100,100,1],[100,-100,1],[-100,100,1]]).T 
    initFormation = np.array([[-180,-180,1],[180,180,1],[180,-180,1],[-180,180,1]]).T  # for 4 robots
    #initFormation = np.array([[-300,-600,1],[300,600,1],[300,-600,1],[-300,600,1]]).T  # for 4 robots

    #initFormation=np.array([[0,-250,1],[0,250,1]]).T
    #initFormation = np.array([[0,0,1],[0,100,1],[0,-100,1],[-100,0,1],[100,0,1],[100,100,1],[-100,-100,1],[100,-100,1],[-100,100,1]]).T  # for 9 robots
    sampleNodes = set()
    # generate the random nodes first
    # put this in node definition.

    sampleNodes = list(sampleNodes)
    visitedNodes = [start]    
    print('Starting RRT*')
    pathLength = 0
    bestPath = None
    for its in tqdm(range(numNodes)):

        # it += 1
        while True:
            #directionNode = Node(min_state + np.random.rand(5) * (max_state - min_state))
            Sxmax2 = max_state[3]**2
            Symax2 = max_state[4]**2
            directionNode0 = np.array(min_state) + np.random.rand(5) * (np.array(max_state) - np.array(min_state))
            S=sample_in_poly(sp)
            directionNode0[3] = np.sqrt(S[0])
            directionNode0[4] = np.sqrt(S[1])
            directionNode = Node(directionNode0)
            loc = directionNode.state.astype(int)
            ### should check coll of direction node here 1
            if dic['newSampling'] and sampledImage[edgeCollStep[1], loc[0]] != 0:
                continue
            if not isColliding(directionNode, obstacleMap, steps = edgeCollStep):
                ### if 1 is free
                directioncoll = False
                distances, closestNodes = findCloseNodes(directionNode, visitedNodes, weights, newDist = newDist)
                # choose closest node as node to extend or parent
                nodeToExtend = closestNodes[0]
                
                if heur: ### this is praveens method to reduce waste samples keep heur false
                    directionNode = adjustDirection(nodeToExtend, directionNode, obstacleMap)
                # extend node to extend in the direction of directionNode
                extendedNode = getNodeExtension(nodeToExtend, directionNode, eps)
                
                # do the edge collision between extended node and node to extend
                #isCollidingnew(extendedNode, obstacleMap,nodeToExtend, steps = edgeCollStep)

                if not isColliding(extendedNode, obstacleMap,nodeToExtend, steps = edgeCollStep):
                    break # breaks only if collision free so that node can be added to tree outside while
                ### till here if free
            else:
                directioncoll = True
        # add node to tree and rest
        if directioncoll == False:
            extendedNode.cost = distance(nodeToExtend, extendedNode, newDist = newDist) + nodeToExtend.cost
            #RRT star code
            distances, neighbourNodes = findCloseNodes(extendedNode, visitedNodes, weights, thresh = neighbourRadius, newDist = newDist)
            for node in neighbourNodes:
                # cost of RRT node
                dista = distance(node, extendedNode, newDist = newDist) + node.cost
                # check cost of other node
                distb = distance(nodeToExtend, extendedNode, newDist = newDist) + nodeToExtend.cost
                if  (dista < distb) and (not isColliding(extendedNode, obstacleMap,node, steps = edgeCollStep)): # better node as per cost
                    # check collision here
                    nodeToExtend = node
                    extendedNode.cost = dista
                    
            #RRT star end
            nodeToExtend.connectChild(extendedNode)
            visitedNodes.append(extendedNode)
            visitedNodes = list(set(visitedNodes))

            ### commented below code on 18-07-2022 not sure of its use seems repeatation of above
            # for node in neighbourNodes:
                
            #     distb = distance(node, extendedNode, newDist = True) + extendedNode.cost
            #     if  (distb < node.cost) and (not isColliding(extendedNode, obstacleMap,node, steps = edgeCollStep)):
            #         node.parent = extendedNode                    
            #         node.cost = distb

            img = render(visitedNodes, goal, start, nodeToExtend, extendedNode, directionNode, obstacleMap)
            show(img,winname='a', waitkey = 1)

            for i in range(len(visitedNodes)):
                visitedNodes[i].id = i

            # closest to goal
            goalDist, goalnodes = findCloseNodes(goal, visitedNodes, np.array([1, 1, 0, 0, 0]), newDist = newDist)
            mgd = min(goalDist)
            print(" "+str(mgd))
            withinGoal = (goalDist < goalRadius) 
            if (sum(withinGoal) > 0) :
                goalnodes = [goalnodes[i] for i in range(len(withinGoal)) if withinGoal[i] == True]
                bestPath = min(goalnodes, key = lambda x: x.cost)
                print('Found goal in', its)
                img = drawBackTrack(bestPath, (img).astype(np.uint8), goal, start)
                #break
        # rest while ends here
    
    if bestPath is None:
        print('Goal not found')
        return -1, -1
    else:
        img = drawBackTrack(bestPath, (img).astype(np.uint8), goal, start)


    #show(img, winname='a', waitkey = 1)
    return its, bestPath.cost, bestPath, img
        

        

if __name__ == "__main__":
    import numpy as np
    import matplotlib.pyplot as plt

    heur = False
    name = f'shortObstacle_heur{heur}'
    
    f = open(f'data/{name}.txt', 'w')

    itss = []
    for i in range(15):
        its, length, node = RRT(heur)
        f.write(f'{its, length}\n')

        itss.append(its)

    fig = plt.hist(itss, bins = 5)
    plt.savefig(f'data/{name}.png')