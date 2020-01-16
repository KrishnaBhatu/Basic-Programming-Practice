from EightPuzzle import EightPuzzle
import numpy as np
import time
'''
Function to check if the puzzle is solvable

return = bool
input param = list(dimention = 1x9)
'''
def invChecker(puzzle):
    result = False
    invCount = 0
    for i in range(len(puzzle)-1):
        for j in range(i+1 ,len(puzzle)):
            if(puzzle[i] > 0 and puzzle[j] > 0 and puzzle[j] < puzzle[i]):
                invCount += 1
    if(invCount%2 == 0):
        result = True
    return result

'''
Function to check if the node with same pattern has occoured before or not

return = bool
input param = list(dynamic dimention) -- The list of already explored nodes
            = EightPuzzle object -- The object of the node that is to be checked
'''
def checker(closedList, currentNode):
    flag = False
    for i in closedList:
        if(i.puzzle == currentNode.puzzle):
            flag = True
            break
        else:
            flag = False
    return flag

'''
Function that does the task of solving the puzzle

return = None
input param = None
'''
def smartSolver():
    print("The format to enter data is as follows: \n")
    print("For inputing the given data \n")
    print("--> 1  2  3\n    4  5  6\n    7  8  0 \n")
    print("Enter the data as --> 1,2,3,4,5,6,7,8,0 ")
    puzzle = input("Enter the 8 puzzle now: ")
    puzzle = list(puzzle)
    print("The puzzle that you have entered is: \n")
    for i in range(len(puzzle)):
            if(i%3 == 0):
                print("\n")
                print(puzzle[i]),
                
            else:
                print(puzzle[i]),
    print("\n")
    #Start solving only if the puzzle is solvable
    if(invChecker(puzzle)):
        start = time.time()  #Start the timer
        timeLimit = True
        nodeNumberCounter = 1
        cost = 0
        startNode = EightPuzzle(puzzle)  # Initialize the root object
        startNode.giveNodeNumber(nodeNumberCounter)
        startNode.myCost(cost)
        nodeToSearch = []  # Initalize a list to store the nodes that are to be searches in the FIFO manner
        nodeAlreadySearched = [] # Initialize the list to store the nodes which are already searched
        allNodes = [] # Initialize all the nodes visited
        nodeToSearch.append(startNode) 
        allNodes.append(startNode)
        goalFound = False
        noNewNode = False
        cost = 1
        # Run the loop until the goal is found and also the nodeToSearch list is exhausted
        while((not goalFound) and (not noNewNode)):
            
            nodeToSearch[0].expandMe() #Calls the method to generate children nodes
            nodeAlreadySearched.append(nodeToSearch[0])
            goalFound = nodeToSearch[0].goalCheck()
            for i in nodeToSearch[0].children:
                cost = (i.getParent().mycost) + 1
                i.myCost(cost)
                if(not checker(nodeAlreadySearched,i)):  #Checks if the node has already occoured or not
                    nodeNumberCounter += 1
                    i.giveNodeNumber(nodeNumberCounter)
                    allNodes.append(i)
                    nodeToSearch.append(i)
            if(len(nodeToSearch) == 0):
                noNewNode = True
            for i in nodeToSearch:
                if(i.goalCheck()):
                    goalNode = i
                    goalFound = True
            del nodeToSearch[0]
            if (time.time() > start + 1500):
                timeLimit = False
                break

        if(timeLimit):
            traceBackEnd = False
            traceBackPath = [] #Initialize the list that stores the path of the goal node
            traceBackPath.append(goalNode)
            temp = traceBackPath[0]
            while(not traceBackEnd): #Stores all the parents starting from the goal node until the root node is seen
                parentPresent = temp.getParent()
                traceBackPath.append(parentPresent)
                temp = parentPresent
                if(parentPresent.checkRoot(puzzle)):
                    traceBackEnd = True
            
            saveNodeInfo(allNodes,puzzle) #Saves NodeInfo and AllVisitedNodes to text file
            savePath(reversed(traceBackPath)) #Saves the path to goal node in a text file
            print("Number of Steps taken: "+ str(len(traceBackPath)-1)+"\n")
            print("Goal Found!")
        else:
            saveNodeInfo(allNodes,puzzle) #Saves NodeInfo and AllVisitedNodes to text file
            savePath(reversed(traceBackPath)) #Saves the path to goal node in a text file
            print("Time Limit Exceeded")
            print("Terminating Program")
    else:
        print("This Problem Cannot be Solved")

'''
Function to store the nodes info and all visited nodes in a text file

return = None
input param = list -- list of all the nodes visited
            = list(1x9) -- root node
'''
def saveNodeInfo(allNodes,root):
    fileNodeInfo = open("NodesInfo.txt", "w")
    fileallNodes = open("Nodes.txt", "w")
    fileNodeInfo.write("NodeNumber, ParentNodeNumber, Cost" + "\n" + "\n")
    for i in allNodes:
        k = 0
        m = -1
        for j in range(len(i.puzzle)):
            if(j%3 == 0):
                k = 0
                m += 1
            if(j == len(i.puzzle)-1):
                fileallNodes.write(str(i.puzzle[m+3*k]) + "\n")
            else:
                fileallNodes.write(str(i.puzzle[m+3*k]) + ",")
            
            k+=1
        if(i.checkRoot(root)):
            fileNodeInfo.write(str(i.nodeNumber) + "," + "0" + "," + str(i.mycost) + "\n")
        else:
            fileNodeInfo.write(str(i.nodeNumber) + "," + str(i.getParent().nodeNumber))
            fileNodeInfo.write("," + str(i.mycost) + "\n")
    fileNodeInfo.close()
    fileallNodes.close()

'''
Function to save path to the goal node

return = None
input param = list --nodes occouring in the path to goal node
'''
def savePath(traceBack):
    filePath = open("nodePath.txt","w")
    for i in traceBack:
        k = 0
        m = -1
        for j in range(len(i.puzzle)):
            if(j%3 == 0):
                k = 0
                m += 1
            if(j == len(i.puzzle)-1):
                filePath.write(str(i.puzzle[m+3*k]) + "\n")
            else:
                filePath.write(str(i.puzzle[m+3*k]) + ",")
            
            k+=1
    filePath.close()

#Calling the function to solve the puzzle
smartSolver()
