#File Description - Path Planning algorithm with GUI
#Author: Krishna S Bhatu
#UID: 116304764

import numpy as np
import cv2
import heapq
import time
import math
##############################################################
#Equations of the obstacle using half plane method
def equationSquare(x,y):
    if((x-50)>=0 and (x-100)<=0 and (y-37.5)>=0 and (y-82.5)<=0):
        return True
    return False

def equationCircle(x,y):
    xN = x - 190
    yN = y - 20
    if((xN**2 + yN**2 - 15**2)<=0):
        return True
    return False

def equationEllipse(x,y):
    xN = x - 140
    yN = y - 30
    if(((xN**2/15**2) + (yN**2/6**2) - 1)<=0):
        return True
    return False

def equationFigure(x,y): 
    if(((y-135)<=0 and ((y-98)+1.85*(x-193))<=0 and (y- 1.64*x + 111)<=0 and (y-98)>=0) or ((y- 1.64*x + 111)<=0 and (y-98)<=0 and (y - 2*(x/19) - (1536/19))>=0) or ((y-98)<=0 and (y + 38*(x/7) - (6880/7))>=0 and (y - 38*(x/23) + (5080/23))>=0)):
        return True
    return False

##############################################################
#Find the obstacles using the above conditions
def isObstacle(i,j):
    if(equationSquare(i,j) or equationCircle(i,j) or equationEllipse(i,j) or equationFigure(i,j)):
        return True
    return False

def findObs():
    obs = set()
    graphPoints = set()
    for i in range(0,251):
        for j in range(0,151):
            tempPoint = (i,(150-j))
            if(isObstacle(i,j)):
                obs.add(tempPoint)
            else:
                graphPoints.add(tempPoint)
    return obs, graphPoints

##############################################################
#Methods for applying Minkowski Difference for the rigid robot
def findRobotCordinates(radiusOfRobot):
    robot = []
    for i in range(-radiusOfRobot,radiusOfRobot+1):
        for j in range(-radiusOfRobot,radiusOfRobot+1):
            if((i**2 + j**2 - radiusOfRobot**2)<=0):
                robot.append((i,j))
    return robot

def findMinkowskiDiff(graphPoints, obstacles, radiusOfRobot):
    newObs = []
    robot = findRobotCordinates(radiusOfRobot)
    for i in obstacles:
        for j in robot:
            tempX = i[0] + (-1)*j[0]
            tempY = i[1] + (-1)*j[1]
            newObs.append((tempX,tempY))
    for i in newObs:
        if i in graphPoints:
            graphPoints.remove(i)
        obstacles.add(i)
    return obstacles, graphPoints

def addWallBoundary(graphPoints, obstacles, radiusOfRobot):
    temp = []
    for i in graphPoints:
        if(i[0] < radiusOfRobot or i[0] > (251-radiusOfRobot) or i[1] < radiusOfRobot or i[1] > (151-radiusOfRobot)):
            obstacles.add(i)
            temp.append(i)
    for i in temp:
        graphPoints.remove(i)
    return obstacles, graphPoints

##############################################################
#Draw the obstacles on the GUI
def drawObstacles(image, obstacles):
    for x in range(0,251):
        for y in range(0,151):
            if((x,y) in obstacles):
                image[(150 - y),x] = (255,0,0)
            else:
                image[(150 - y),x] = (255,255,255)
    return image

##############################################################
#Method for Dijkstra Algorithm
def checkInminPQ(minpq, checkNode, cost, parentChildList, currentNode):
    matchFound = False
    isLessThanCurrent = False
    itr = 0
    for i in range(0,len(minpq)):
        if(checkNode in minpq[i]):
            matchFound = True
            if(cost < minpq[i][0]):
                itr = i
                isLessThanCurrent = True
                break
    if(matchFound):
        if(isLessThanCurrent):
            minpq[itr] = minpq[-1]
            minpq.pop()
            heapq.heapify(minpq)
            parentChildList.update({checkNode:currentNode})
            heapq.heappush(minpq, (cost, checkNode))
    elif(not matchFound):
        heapq.heappush(minpq, (cost, checkNode))               #Changed this
    return minpq, parentChildList
def dijkstraAlgorithm(startNode, goalNode, graphPoints, image, pathIsPossible):
    minpq = []
    visitedNodes = {}
    parentChildList = {}    ####    me, parent

    cost = 0
    heapq.heappush(minpq, (cost, startNode))
    parentChildList.update({(startNode):(startNode)})

    currentNode = startNode
    counter = 0 #remove this
    while(currentNode != goalNode and pathIsPossible):
        counter = counter + 1   #remove this
        currentcost, currentNode = heapq.heappop(minpq)
        visitedNodes.update({currentNode:currentcost})
        
        for i in range(-1,2):
                for j in range(-1,2):
                    if(not (i == 0 and j == 0)):
                        checkNode = (currentNode[0]+i,currentNode[1]+j)
                        if (checkNode in graphPoints) :
                            if(abs(i-j) == 1):
                                selfCost = 1
                                if(not(checkNode in parentChildList.keys())):
                                    parentChildList.update({checkNode:currentNode})                                
                                totalCost = selfCost + visitedNodes[currentNode]
                                
                                if(checkNode in visitedNodes.keys()):
                                    continue
                                else:
                                    minpq, parentChildList = checkInminPQ(minpq, checkNode, totalCost, parentChildList, currentNode)
                                
                            else:
                                selfCost = 1.4
                                if(not(checkNode in parentChildList.keys())):
                                    parentChildList.update({checkNode:currentNode})
                                totalCost = selfCost + visitedNodes[currentNode]

                                if(checkNode in visitedNodes.keys()):
                                    continue
                                else:
                                    minpq, parentChildList = checkInminPQ(minpq, checkNode, totalCost, parentChildList, currentNode)
        if(exploration):
            image[(150 - currentNode[1]),currentNode[0]] = (0,255,255)
            cv2.imshow('Obs', image)    
            cv2.waitKey(1)
        if(len(minpq)==0 and currentNode != goalNode):
            print("Path cannot be found!")
            pathIsPossible = False

    return visitedNodes, parentChildList, image, pathIsPossible

##############################################################
#Method for A Star Algorithm
def checkInminPQA(minpq, checkNode, cost, parentChildList, currentNode):
    matchFound = False
    isLessThanCurrent = False
    itr = 0
    for i in range(0,len(minpq)):
        if(checkNode in minpq[i]):
            matchFound = True
            if(cost < minpq[i][0]):
                itr = i
                isLessThanCurrent = True
                break
    if(matchFound):
        if(isLessThanCurrent):
            minpq[itr] = minpq[-1]
            minpq.pop()
            heapq.heapify(minpq)
            parentChildList.update({checkNode:currentNode})
            heapq.heappush(minpq, (cost, checkNode))
    elif(not matchFound):
        heapq.heappush(minpq, (cost, checkNode))
        parentChildList.update({checkNode:currentNode})               
    return minpq, parentChildList

def findHcost(currentNode, goalNode):
    cost = math.sqrt((currentNode[0]-goalNode[0])**2 + (currentNode[1]-goalNode[1])**2)
    return cost

def aStarAlgorithm(startNode, goalNode, graphPoints, image, pathIsPossible):
    minpq = []
    visitedNodes = {}
    parentChildList = {}    ####    me, parent
    nodeCcost = {}
    cost = 0
    heapq.heappush(minpq, (cost, startNode))
    parentChildList.update({(startNode):(startNode)})
    currentcost, currentNode = heapq.heappop(minpq)
    nodeCcost.update({currentNode:cost})
    counter = 0
    currentNode = startNode
    while(currentNode != goalNode and pathIsPossible):
        counter = counter + 1
        visitedNodes.update({currentNode:currentcost})
        
        for i in range(-1,2):
                for j in range(-1,2):
                    if(not (i == 0 and j == 0)):
                        checkNode = (currentNode[0]+i,currentNode[1]+j)
                        if (checkNode in graphPoints) :
                            if(checkNode not in visitedNodes.keys()):
                                if(abs(i-j) == 1):
                                    selfCost = 1
                                    hCost = findHcost(checkNode, goalNode)
                                    cCost = selfCost + nodeCcost[currentNode]   #cCost is dependent on the closest parent, high c cost will not change the parent
                                    
                                    
                                    totalCost = cCost + hCost
                                    minpq, parentChildList = checkInminPQA(minpq, checkNode, totalCost, parentChildList, currentNode)

                                        

                                else:
                                    selfCost = 1.4
                                    hCost = findHcost(checkNode, goalNode)
                                    cCost = selfCost + nodeCcost[currentNode]   #cCost is dependent on the closest parent, high c cost will not change the parent
                                    
                                    totalCost = cCost + hCost
                                    minpq, parentChildList = checkInminPQA(minpq, checkNode, totalCost, parentChildList, currentNode)


        currentLowestFcost = minpq[0][0]
        nextNodePosition = 0
        minFcost = 100000
        for i in range(0, len(minpq)):
            if(currentLowestFcost in minpq[i]):
                fCostTemp = findHcost(minpq[i][1], goalNode)
                
                if(minFcost > fCostTemp):
                    nextNodePosition = i
                    minFcost = fCostTemp
        currentcost, currentNode = minpq[nextNodePosition]
        minpq[nextNodePosition] = minpq[-1]
        minpq.pop() 
        heapq.heapify(minpq)
        currCcost = currentcost - findHcost(currentNode, goalNode)
        nodeCcost.update({currentNode:currCcost})
        if(currentNode == goalNode):
            visitedNodes.update({currentNode:currentcost})
        
        if(exploration):
            image[(150 - currentNode[1]),currentNode[0]] = (0,255,255)
            cv2.imshow('Obs', image)    
            cv2.waitKey(1)

        if(len(minpq) == 0 and currentNode != goalNode):
            print("Path cannot be found!")
            pathIsPossible = False

    return visitedNodes, parentChildList, image, pathIsPossible

##############################################################
#Code Logic for initialization and taking Input 
pathIsPossible = True
properFormat = False
pointRobot = False
rigidRobot = False
while(not properFormat):
    robotInput = input("Do you want to the robot to be point or rigid ? (answer r for rigid and p for point) ")
    if(robotInput == 'p' or robotInput == 'P'):
        pointRobot = True
        properFormat = True
    elif(robotInput == 'r' or robotInput == 'R'):
        rigidRobot = True
        properFormat = True
    else:
        print("Enter in proper format")

if(pointRobot and (not rigidRobot)):
    radiusOfRobot = 0
    clearance = 0
elif(rigidRobot and (not pointRobot)):
    radiusOfRobot = int(input("Enter the radius of robot: "))
    clearance = int(input("Enter the clearance: "))
newImage = np.zeros((151, 251, 3), np.uint8)
obstacles, graphPoints = findObs()  
radiusOfRobot = radiusOfRobot + clearance
obstacles, graphPoints = findMinkowskiDiff(graphPoints, obstacles, radiusOfRobot)
obstacles, graphPoints= addWallBoundary(graphPoints, obstacles, radiusOfRobot)
image = drawObstacles(newImage, obstacles)

properFormat = False
while(not properFormat):
    explorationInput = input("Do you want to see exploration of nodes ? (answer y or n) ")
    if(explorationInput == 'y' or explorationInput == 'Y'):
        exploration = True
        properFormat = True
    elif(explorationInput == 'n' or explorationInput == 'N'):
        exploration = False
        properFormat = True
    else:
        print("Enter in proper format")

startNode = tuple(map(int,input("Enter the start node: ").split(',')))
while(startNode in obstacles):
    startNode = tuple(map(int,input("Enter the start node: ").split(',')))
goalNode = tuple(map(int,input("Enter the goal node: ").split(',')))
while(goalNode in obstacles):
    goalNode = tuple(map(int,input("Enter the goal node: ").split(',')))


properFormat = False
astar = False
dijkstra = False
while(not properFormat):
    algoInput = input("Which algorithm do you want to use A* or Dijkstra ? (answer a for A* and d for Dijkstra) ")
    if(algoInput == 'a' or algoInput == 'A'):
        astar = True
        properFormat = True
    elif(algoInput == 'd' or algoInput == 'D'):
        dijkstra = True
        properFormat = True
    else:
        print("Enter in proper format")


starttime = time.time()
##############################################################
#Call for algorithm to find the path from start to end node
if(dijkstra):
    visitedNodes, parentChildList, image, pathIsPossible = dijkstraAlgorithm(startNode, goalNode, graphPoints, image, pathIsPossible)
elif(astar):
    visitedNodes, parentChildList, image, pathIsPossible = aStarAlgorithm(startNode, goalNode, graphPoints, image, pathIsPossible)

##############################################################
#Draw the final path
if(pathIsPossible):
    nodeNow = goalNode
    while(nodeNow != startNode):
        nodeNow = parentChildList[nodeNow]
        image[(150 - nodeNow[1]), nodeNow[0]] = (0,0,255)
    print("Total cost of the path", visitedNodes[goalNode])
    print("Time taken to run in seconds", time.time() - starttime)
##############################################################
cv2.imshow('Obs', image)    
cv2.waitKey(0)
cv2.destroyAllWindows()