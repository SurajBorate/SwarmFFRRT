import numpy as np
import cv2
from rrt2d import RRT, drawBackTrack, isColliding
#from rrtSmooth import RRTSmooth, isColliding2
from LPsigma import *
from randompoly import *
from utils import *
global locations

np.random.seed(2)

width = 3000
height = 2000
maxrobotrad = 160
markersqbytwo = 96
obssize = 500

obstacleMap = np.zeros((height, width), np.uint8)
#initFormation = np.array([[-300,-600,1],[300,600,1],[300,-600,1],[-300,600,1]]).T  # for 4 robots
#initFormation = np.array([[0,-250,1],[0,250,1]]).T  # for 2 robots
#initFormation = np.array([[0,0,1],[0,100,1],[0,-100,1],[-100,0,1],[100,0,1],[100,100,1],[-100,-100,1],[100,-100,1],[-100,100,1]]).T  # for 9 robots
initFormation = np.array([[-200,-200,1],[200,200,1],[200,-200,1],[-200,200,1]]).T  # for 4 robots

rad = 160
poly1 = find_Scaling_Poly(initFormation,rad,1.5,1.5,False) 
sp= sort_points_clock(poly1)
# top block for Narrow case
#obstacleMap[ 0:(obssize+maxrobotrad),int(width/2)-int(obssize/2)-maxrobotrad:int(width/2+obssize/2+maxrobotrad), ] = 255
# bottom block for Narrow case
#obstacleMap[ height-obssize-maxrobotrad:height,int(width/2)-int(obssize/2)-maxrobotrad:int(width/2)+int(obssize/2)+maxrobotrad, ] = 255

# center block for Split Merge case
#obstacleMap[ int(height/2)-int(obssize/2)-maxrobotrad:int(height/2)+int(obssize/2)+maxrobotrad,int(width/2)-int(obssize/2)-maxrobotrad:int(width/2)+int(obssize/2)+maxrobotrad, ] = 255
#obstacleMap[502*2-int(obssize/2)-maxrobotrad:502*2+int(obssize/2)+maxrobotrad,531*3-int(obssize/2)-maxrobotrad:531*3+int(obssize/2)+maxrobotrad, ] = 255

# two blocks   ::: 189 to 120  632::500
obstacleMap[0:100*2+int(obssize/2)+maxrobotrad,490*3-int(obssize/2)-maxrobotrad:490*3+int(obssize/2)+maxrobotrad, ] = 255
obstacleMap[2000-100*2-int(obssize/2)-maxrobotrad:2000,490*3-int(obssize/2)-maxrobotrad:490*3+int(obssize/2)+maxrobotrad, ] = 255

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


# obstacleMap[ 260:390, :10] = 255
# obstacleMap[ 260:390, 70:,] = 255
# obstacleMap[200:, :130] = 255

# obstacleMap[290:340, 270:] = 0
# obstacleMap[340:390, 270:330] = 0
# cv2.circle(obstacleMap, (135, 300), 100, 0, -1)
# cv2.circle(obstacleMap, (160, 310), 60, 255, -1)

ogObstacleMap = obstacleMap.copy()

def generateDistFromOGM(ogm):
    padding = 300
    overallmap = np.ones((height + padding, width + padding), np.uint8)*255
    
    overallmap[padding//2:height + padding//2, padding//2:width + padding//2] = ogm
    dist = cv2.distanceTransform(255 - overallmap, cv2.DIST_L2, 5)
    dist = -dist[padding//2:height + padding//2, padding//2:width + padding//2]

    dist -= dist.min()
    # dist[obstacleMap == 255] = dist.max() * 10

    return dist


dist = generateDistFromOGM(obstacleMap)

# start = Node([500, height/2, 0, 2, 2])
start = Node([600, 1000, 0,  2.3, 3])
maxrobotsize = 300
# goal = Node([400, 75, 0, 1, 1])
goal = Node([width-500,height/2+200, 0, 1, 1])
# goal = Node([480, 250, 0, 1, 1])
# goal = Node([480, 315, 0, 1, 1])

dic = {
    'maxDistanceExtend' : 50, # max step size
    'nAgents' : 4,
    'newSampling' : False,
    'neighbourRadius':1000, # RRT star for RRT turn it to zero
    'newDist' : True, # new dist function
    'heur' : False, # new sampling to increase speed
    'radius' : 160, # robot size
    'start' : start,
    'obstacleMap' : obstacleMap,
    'heur' : False,
    'goal' : goal,
    'goalRadius' : 200, # where to stop from goal
    'max_state' : np.array([width - 160, height - 160, 2*np.pi,1, 1]),
    'min_state' : np.array([150, 150, 0, 0.2, 0.2]),
    'sortpoly':sp,
    'edgeCollStep':30
}

show(dist, waitkey= 0)

its, length, bestPath, img = RRT(dic)
_, waypoints = drawBackTrack(bestPath, img, None, None, True)
print(waypoints)

   
np.save('waypoint_arraynew',waypoints)
print("Done")
show(img, waitkey= 0)

# def smooth(viz = False):
#     global obstacleMap, bestPath
#     if viz:
#         save(img, 'beforeSmooth')

#     rope = convertTreeToRope(bestPath, slack = 1)
#     vizimg = cv2.cvtColor(dist, cv2.COLOR_GRAY2BGR)

#     if viz:
#         save(dist, 'gradientmap')

#     vizimg = rope.isConverged(dist, vizimg, viz = viz)

#     nodes = convertRopeToTree(rope)
#     bestPath = nodes[-1]

    
#     pathImg = cv2.cvtColor(obstacleMap.copy(), cv2.COLOR_GRAY2BGR)
#     pathImg, waypoints = drawBackTrack(bestPath, pathImg, None, None, True)

#     return bestPath, pathImg

# #smooth()
# #show(pathImg, 'b', waitkey = 0)

# for i in range(300):
#     center = (200, -25 + 5*i)
#     obstacleMap = ogObstacleMap.copy()
#     cv2.circle(obstacleMap, center, 25, 255, -1)
#     thisObstacleMap = obstacleMap.copy()
#     dist = generateDistFromOGM(obstacleMap)
#     bestPath, pathImg = smooth(False)

#     node = bestPath
#     collisionNodes = []
#     prevnode = bestPath
#     while node is not None:
#         if isColliding(node, thisObstacleMap):
#             collisionNodes.append(prevnode)
        
#         prevnode = node
#         node = node.parent

#     if len(collisionNodes) > 0:
#         dic['start'] = collisionNodes[-1].parent
#         dic['goal'] = collisionNodes[0]
#         dic['obstacleMap'] = thisObstacleMap
#         its, length, stitchPath, img = RRT(dic)
#         collisionNodes[0].parent = stitchPath

#         bestPath, pathImg = smooth(False)

#     #show(pathImg, 'b', waitkey = 1)
    
        


