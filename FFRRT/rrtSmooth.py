from os import close
import numpy as np
import cv2
from tqdm import tqdm

from rrt2d import Node

def angDist(ang1, ang2):
    maxi = max(ang1, ang2)
    mini = min(ang1, ang2)
   
    diff = maxi - mini
    
    if diff <=180:
        dist_angle = diff
    
    if diff > 180:
        dist_angle = 360 - diff

    return dist_angle

def distance(nodeA, nodeB, weights = np.array([0, 0, 1/3, 1/3, 1/3]), useMap = False):
    ang1 = nodeA.state[2]
    ang2 = nodeB.state[2]
    
    dist = (nodeA - nodeB)
    dist = dist.state / (dist.norm() + 1e-6)
    
    dist[2] = angDist(ang1, ang2)/(2*np.pi)

    return np.sqrt(np.dot((nodeA.state - nodeB.state)**2, weights))

    
def findCloseNodes(node, nodes, weights, thresh = float('inf')):
    nodes = list(nodes)
    distances = []
    finalnodes = []
    for i in range(len(nodes)):
        dist = distance(node, nodes[i], weights)
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

    return directionNode


def getNodeExtension(node, directionNode, eps):
    delta = (directionNode - node)
    val = distance(directionNode, node, useMap= True)
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
    #cv2.circle(img, (int(goal.state[0]), int(goal.state[1])), goalRadius, (0, 0, 128), -1)

    color = (100, 255, 50)

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
    drawFormation(start, img, color = (0, 0, 255))

    cv2.line(img, (int(extendedNode.state[0]), int(extendedNode.state[1])), (int(nodeToExtend.state[0]), int(nodeToExtend.state[1])), (0, 0, 200), 1)
    
    return img

def drawFormation(node, img, color = (255, 255, 255), verbose = False, prevNode = None):
    global initFormation
    if prevNode is not None:
        newLocs = getPosesFromFormation(prevNode)

    locs = getPosesFromFormation(node)
    colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
    for i, loc in enumerate(locs):
        cv2.circle(img, (int(loc[0]), int(loc[1])), 5, colors[i % 4], -1)
        if prevNode is not None:
            cv2.line(img, (int(loc[0]), int(loc[1])), (int(newLocs[i][0]), int(newLocs[i][1])), colors[i % 4], 1)
            cv2.circle(img, (int(newLocs[i][0]), int(newLocs[i][1])), 5, colors[i % 4], -1)

    if verbose:
        print(locs)
        print(node.state)

    return img

    

def drawBackTrack(node, img, goal, start, obstacleMap):
    img = cv2.cvtColor(obstacleMap.copy(), cv2.COLOR_GRAY2BGR)

    color = (0, 255, 0)
    if goal is not None:
        cv2.circle(img, (int(goal.state[0]), int(goal.state[1])), goalRadius, (255, 0, 0), -1)
        img = goal.draw(img, (255, 0, 0))
    if start is not None:
        img = start.draw(img, (0, 255, 0))
    while node is not None:
        img = node.draw(img, (255, 255, 0))
        img = (img * 0.98).astype(np.uint8)
        parent = node.parent
        drawFormation(node, img, color = (0, 255, 0), prevNode = parent)
        if parent is not None:
            cv2.line(img, (int(node.state[0]), int(node.state[1])), (int(parent.state[0]), int(parent.state[1])), (0, 0, 200), 1)
        show(img, waitkey=50)
        
        node = parent

    return img

def show(img, winname = 'a', waitkey = 0):
    cv2.imshow(winname, img)

    if cv2.waitKey(waitkey) == ord('q'):
        exit()

def getPosesFromFormation(node):
    global initFormation
    xc, yc, theta, sx, sy = node.state 
    theta = theta * np.pi / 180
    matr = np.array(
        [
            [sx * np.cos(theta), -sy * np.sin(theta), xc],
            [sx * np.sin(theta), sy * np.cos(theta), yc],
            [0, 0, 1]
        ]
    )

    locs = np.dot(matr, initFormation)
    locs = locs[:2].T.astype(np.int)
    return locs


def isColliding2(node, obstacleMap):
    # x, y = int(node.state[0]), int(node.state[1])
    locs = getPosesFromFormation(node)
    for loc in locs:
        try:
            if obstacleMap[loc[1], loc[0]] == 255:
                return True
        except:
            continue
    
    return False

def RRTSmooth(endNode, obstacleMap):
    global initFormation
    global visitedNodes, sampledImage
    global goalRadius

    numNodes = 1000

    x_min = 10
    y_min = 10
    y_max = 490
    x_max = 490

    eps = 10
    goalRadius = 40
    neighbourRadius = 25

    min_scale = np.array([0.1,0.1])
    max_scale = np.array([1, 1])
    
    initFormation = np.array([
        [60 * np.cos(theta+ np.pi/4 ), 60 * np.sin(theta + np.pi/4), 1] for theta in np.arange(0, 2*np.pi, 2*np.pi/5)
    ]).T

    #initFormation = np.array([[-100,-100,1],[100,100,1],[100,-100,1],[-100,-100,1]]).T
    initFormation = np.array([[0,-250,1],[0,250,1]]).T 
    

    sampleNodes = set()

    sampleNodes = list(sampleNodes)

    weights = np.array([0, 0, 1, 1, 1])
    weights = weights / np.sum(weights)

    print('Smoothing search')

    bestPath = None
    temp = endNode
    visitedNodes = [endNode]
    
    while endNode is not None:
        if not isColliding(endNode, obstacleMap):
            if endNode.parent is None:
                break
            endNode.parent.state[2:] = endNode.state[2:]
            endNode = endNode.parent
            continue
        while True:
            directionNode = Node([
                endNode.state[0],
                endNode.state[1],
                np.floor(np.random.rand()*360),
                min_scale[0] + np.random.rand()*(max_scale[0] - min_scale[0]),
                min_scale[1] + np.random.rand()*(max_scale[1] - min_scale[1])
            ])

            distances, closestNodes = findCloseNodes(directionNode, visitedNodes, weights)

            nodeToExtend = closestNodes[0]

            extendedNode = getNodeExtension(nodeToExtend, directionNode, eps)

            if not isColliding(extendedNode, obstacleMap):
                break

        extendedNode.cost = distance(nodeToExtend, extendedNode) + nodeToExtend.cost

        distances, neighbourNodes = findCloseNodes(extendedNode, visitedNodes, weights, thresh = neighbourRadius)
        for node in neighbourNodes:
            dista = distance(node, extendedNode) + node.cost
            distb = distance(nodeToExtend, extendedNode) + nodeToExtend.cost
            if  dista < distb:
                nodeToExtend = node
                extendedNode.cost = dista

        nodeToExtend.connectChild(extendedNode)

        visitedNodes.append(extendedNode)
        visitedNodes = list(set(visitedNodes))


        for node in neighbourNodes:
            
            distb = distance(node, extendedNode) + extendedNode.cost
            if  distb < node.cost:
                node.parent = extendedNode
                
                node.cost = distb
        endNode.state[2:] = extendedNode.state[2:]
        endNode = endNode.parent

    return temp
        

        