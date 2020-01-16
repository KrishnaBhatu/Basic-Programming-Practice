import copy

'''
Class defining a Tree data structure which holdes all the nodes as its object
Class information
Parameters:
    children --> dtype = list(dynamic dimention)
                 usage = Holds the children nodes of the given object

    puzzle --> dtype = list(dimention = 1x9)
               usage = Holds the pattern of 8 puzzle for that node

    positionZero --> dtype = int
                     usage = Stores the location of zero in the puzzle

    parent --> dtype = EightPuzzle object
               usage = Stores the parent node object

    nodeNumber --> dtype = int
                   usage = Stores the unique number of the node

    myCost --> dtype = int
               usage = Stores the cost to reach the node
               
Methods:
    constructor --> return = None
                    input param = list(dimention = 1x9)
                    usage = Required for object initialization where the puzzle pattern is stored and the children list is initialised
                    
    printMe --> return  = None
                input param = None
                usage = Prints the 8 puzzle pattern of the node in the form of 3x3 matrix

    findZero --> return = None
                 input param = None
                 usage = Finds and stores the position of zero for that node object

    copyParent --> return = list(dimention = 1x9)
                            A deep copy of the the 8 puzzle pattern of the current node(copies the puzzle parameter)
                   input param = None
                   usage = Used to get a copy of the pattern of the parent node

    goalCheck --> return = bool
                  input param = None
                  usage = Checks if the current node is the goal node and if yes then it returns True

    moveLeft --> return = None
                 input param = None
                 usage = Used to create a child which is made by moving the zero to it's left.
                         Also, it stores the child object in the children list and the current node in the parent parameter of the child node

    moveRight --> return = None
                 input param = None
                 usage = Used to create a child which is made by moving the zero to it's right.
                         Also, it stores the child object in the children list and the current node in the parent parameter of the child node

    moveUp --> return = None
                 input param = None
                 usage = Used to create a child which is made by moving the zero to it's up.
                         Also, it stores the child object in the children list and the current node in the parent parameter of the child node

    moveDown --> return = None
                 input param = None
                 usage = Used to create a child which is made by moving the zero to it's down.
                         Also, it stores the child object in the children list and the current node in the parent parameter of the child node

    expandMe --> return = None
                 input param = None
                 usage = Performs the task of calling the methos like findZero, moveRight, moveLeft, moveUp and moveDown. Thus exploring the node in the Tree.

    getParent --> return = EightPuzzle object
                           The parent node of the current node
                  input param = None
                  usage = Gives the parentn node

    checkRoot --> return = bool
                  input param = None
                  usage = Checks is the current node is a root node

    giveNodeNumber --> return = None
                       input param = int
                       usage = Stores the node number, unique for each node

    isMyChild --> return = bool
                  input param = EightPuzzle object
                                The node that needs to be check
                  usage = Checks if the input node is the child for the currnt node

                         
    myCost --> return = None
               input param = int
               usage = Stores the cost of the node to reach it

'''

class EightPuzzle:
    def __init__(self,  mystate):
        self.children = []
        self.puzzle = mystate

    def printMe(self):
        for i in range(len(self.puzzle)):
            if(i%3 == 0):
                print("\n")
                print(self.puzzle[i]),
                
            else:
                print(self.puzzle[i]),
    
    def findZero(self):
        self.positionZero = self.puzzle.index(0) + 1

    def copyParent(self):
        return copy.deepcopy(self.puzzle)

    def goalCheck(self):
        goalPuzzle = [1,2,3,4,5,6,7,8,0]
        return(self.puzzle == goalPuzzle)

    def moveRight(self):
        childPuzzle = self.copyParent()
        if(self.positionZero%3 != 0):
            childPuzzle[self.positionZero - 1], childPuzzle[self.positionZero] = childPuzzle[self.positionZero], childPuzzle[self.positionZero - 1]
            child = EightPuzzle(childPuzzle)
            self.children.append(child)
            child.parent = self
    def moveLeft(self):
        childPuzzle = self.copyParent()
        if((self.positionZero)%3 != 1):
            childPuzzle[self.positionZero - 2], childPuzzle[self.positionZero - 1] = childPuzzle[self.positionZero - 1], childPuzzle[self.positionZero - 2]
            child = EightPuzzle(childPuzzle)
            self.children.append(child)
            child.parent = self
    def moveUp(self):
        childPuzzle = self.copyParent()
        if(self.positionZero > 3):
            childPuzzle[self.positionZero - 4], childPuzzle[self.positionZero - 1] = childPuzzle[self.positionZero - 1], childPuzzle[self.positionZero - 4]
            child = EightPuzzle(childPuzzle)
            self.children.append(child)
            child.parent = self
    def moveDown(self):
        childPuzzle = self.copyParent()
        if(self.positionZero < 7):
            childPuzzle[self.positionZero + 2], childPuzzle[self.positionZero - 1] = childPuzzle[self.positionZero - 1], childPuzzle[self.positionZero + 2]
            child = EightPuzzle(childPuzzle)
            self.children.append(child)
            child.parent = self
    
    def expandMe(self):
        self.findZero()
        self.moveRight()
        self.moveLeft()
        self.moveUp()
        self.moveDown()

    def getParent(self):
        return self.parent

    def checkRoot(self,root):
        return(self.puzzle == root)

    def giveNodeNumber(self, number):
        self.nodeNumber = number

    def isMyChild(self, node):
        flag = False
        for i in self.children:
            if(i == node):
                flag = True
        return flag

    def myCost(self, cost):
        self.mycost = cost
