import cab320_search

import cab320_sokoban
import thread
import threading
import time, os, multiprocessing

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class SokobanPuzzle(cab320_search.Problem):
    '''
    Class to represent a Sokoban puzzle.
    Your implementation should be compatible with the
    search functions of the module  cab320_search
    '''
    def __init__(self, puzzleFileName):
        self.warehouse = cab320_sokoban.Warehouse()
        self.warehouse.read_warehouse_file(puzzleFileName)
        self.taboo = tabooTuple(self.warehouse.walls,self.warehouse.targets)
        self.initial = (self.warehouse.worker,tuple(self.warehouse.boxes))
    def actions(self, state):
        """
        Builds a list of actions that can be taken from a given state. Actions
        are string variables that correspond to cardinal directions possible
        for the worker to move.
        state: tuple list of form ((x_w,y_w),((x1,y1),(x2,y2)...(xn,yn)))
        state[0] corresponds to the worker location
        state[1] corresponds to a list of box locations
        returns: string list of possible directions to move the worker
        ['Left','Right','Up','Down']
        """
        worker = state[0]
        boxes = state[1]
        walls = self.warehouse.walls
        taboo = self.taboo

        possibleMoves = {(0, -1):'Up',(0, 1):'Down',(-1, 0):'Left',(1, 0):'Right'}
        legalMoves = []
        for direction in possibleMoves.keys():
            checkPos = addCoords(direction,worker)
            if checkPos not in walls and checkPos not in boxes:
                legalMoves.append(possibleMoves[direction])
            if checkPos in boxes:
                checkPos = addCoords(direction,checkPos)
                if checkPos not in list(boxes)+taboo+walls:
                    legalMoves.append(possibleMoves[direction])
        return legalMoves
    def goal_test(self,state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal, as specified in the constructor. Override this
        method if checking against a single self.goal is not enough."""
        return set(self.warehouse.targets)==set(state[1])



    def result(self, state, action):
        """
        Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state).
        """
        assert action in self.actions(state)
        worker = state[0]
        boxes = state[1]
        newState = []
        possibleMoves = {'Up':(0, -1),'Down':(0, 1),'Left':(-1, 0),'Right':(1, 0)}
        worker = addCoords(worker,possibleMoves[action])
        newBoxes = []
        for box in boxes:
            tempBox = box
            if worker == box:
                tempBox = addCoords(box,possibleMoves[action])
            newBoxes.append(tempBox)

        return (worker,tuple(newBoxes))


    def path_cost(self,c,state1,action,state2):
        """Return the cost of a solution path that arrives at state2 from
        state1 via action, assuming cost c to get up to state1. If the problem
        is such that the path doesn't matter, this function will only look at
        state2.  If the path does matter, it will consider c and maybe state1
        and action. The default method costs 1 for every step in the path."""
        return c + 1

    def print_solution(self, goal_node):
        """
            Shows solution represented by a specific goal node.
            For example, goal node could be obtained by calling
                goal_node = breadth_first_tree_search(problem)
        """
        path = goal_node.path()
        print "Solution takes {0} steps from the initial state\n".format(len(path)-1)
        self.warehouse.worker = path[0].state[0]
        self.warehouse.boxes = path[0].state[1]
        print self.warehouse.visualize()
        print "to the goal state\n"
        self.warehouse.worker = path[-1].state[0]
        self.warehouse.boxes = path[-1].state[1]
        print self.warehouse.visualize()
        print "Below is the sequence of moves\n"

        print self.return_path(path)

    def return_path(self,path):
        actionSequence = []
        for node in path:
            if node.action is not None:
                actionSequence.append(node.action)
        return actionSequence

    def h(self,node):
        h = 0
        worker = node.state[0]
        boxes = node.state[1]
        targets = self.warehouse.targets
        for box in boxes:
            if box not in targets:
                closestGoal = closestPosition(box,targets)
                h = h + manhattanDistance(box,closestGoal)
        closestBox = closestPosition(worker,boxes)
        return h + manhattanDistance(closestBox,worker)


class SokobanPuzzleMacro(SokobanPuzzle):
    name = 'Macro Solver'
    def actions(self, state):
        """
        Builds a list of actions that can be taken from a given state. Actions
        are string variables that correspond to cardinal directions possible
        for the worker to move.
        state: tuple list of form ((x_w,y_w),((x1,y1),(x2,y2)...(xn,yn)))
        state[0] corresponds to the worker location
        state[1] corresponds to a list of box locations
        returns: string list of possible directions to move the worker
        ['Left','Right','Up','Down']
        """
        worker = state[0]
        boxes = state[1]
        walls = self.warehouse.walls
        taboo = self.taboo
        legalMoves = []
        count = 0
        boxMove = 0
        elementaryMoves = {(0, -1):'U',(0, 1):'D',(-1, 0):'L',(1, 0):'R'}
        for move in elementaryMoves:
            extendWorker = addCoords(move,worker)
            while extendWorker not in walls and boxMove is not 2:
                if extendWorker in boxes:
                    boxMove = boxMove + 1
                count = count + 1
                extendWorker = addCoords(move,extendWorker)
            count = count - boxMove
            for i in range(count):
                tempString = ''
                for j in range(i+1):
                    tempString = tempString + elementaryMoves[move]
                if tempString is not '':
                    legalMoves.append(tempString)
            count = 0
            secondBox = False
            boxMove = 0
        return legalMoves


    def result(self, state, action):
        """
        Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state).
        """
        assert action in self.actions(state)
        worker = state[0]
        boxes = state[1]
        newState = []
        possibleMoves = {'U':(0, -1),'D':(0, 1),'L':(-1, 0),'R':(1, 0)}
        newBoxes = list(boxes)
        count = 0
        for move in action:
            count += 1
        totalMove = tuple([x*count for x in possibleMoves[action[0]]])
        newWorker = addCoords(worker,totalMove)
        newBoxes = []
        for box in boxes:
            temp = box
            if inMovement(box,worker,possibleMoves[action[0]],count):
                temp = addCoords(newWorker,possibleMoves[action[0]])
            newBoxes.append(temp)

        print tuple(newBoxes)
        return (newWorker,tuple(newBoxes))

    def h(self,node):
        h = 0
        worker = node.state[0]
        boxes = node.state[1]
        targets = self.warehouse.targets
        for box in boxes:
            h = h + 2 #assume it will take 2 macro moves to move box to target
            for target in targets:
                if isInLine(box,target) is not None:
                    #assumption is wrong and therefore only needs 1 move to get
                    #box to target
                    h = h - 1
        closestBox = closestPosition(worker,boxes)
        if isInLine(worker,closestBox) is not None:
            h = h + 1 #worker only 1 away from box, add 1
        else:
            h = h + 2
        return h
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def inMovement(pos1,pos2,movement,steps):
    for i in range(steps):
        pos1 = addCoords(pos1,movement)
        if pos1 == pos2:
            return True
    return False
def closestPosition(pos1,targets):
    '''
    Finds the closest position amongst a list of possible targets and a given
    position
    pos1: tuple of form (x,y)
    targets: list of tuples of form ((x1,y1),(x2,y2),..(xn,yn))
    return: closest tuple pair between pos1 and targets via manhattan distance
    '''
    distance = 10000000 #start with impossible number and then reduce
    closest = []
    for target in targets:
        newDistance = manhattanDistance(pos1,target)
        if distance > newDistance:
            closest = target
            distance = newDistance
    return closest

def manhattanDistance(pos1,pos2):
    '''
    Finds the manhattan distance between 2 tuple pairs
    pos1: tuple of form (x1,y2)
    pos2: tuple of form (x1,y2)
    returns: manhattan distance |x1-x2| + |y1-y2|
    '''
    return abs((pos1[0]-pos2[0])) + abs((pos1[1]-pos2[1]))

def checkActions(puzzleFileName, actionSequence):
    '''
    This is a function called by the automatic marker.

    Your implementation should load a Sokoban puzzle from a text file,
    then try to apply the sequence of actions listed in actionSequence.

    @param puzzleFileName: file name of the puzzle
         (same format as for the files in the warehouses directory)
    @param actionSequence: a sequence of actions.
           For example, ['Left', 'Down', Down','Right', 'Up', 'Down']
    @return:
        The string 'Failure', if one of the move was not successul.
           For example, if the agent tries to push two boxes,
                        or push into to push into a wall.
        Otherwise, if all moves were successful return
               A string representing the state of the puzzle after applying
               the sequence of actions.  This should be the same string as the
               string returned by the method  WarehouseHowever.visualize()
    '''
    puzzle = SokobanPuzzle(puzzleFileName)
    for action in actionSequence:
        newAction = action[0]
        if newAction not in puzzle.actions(puzzle.warehouse):
            return 'Failure'
        puzzle.warehouse = puzzle.result(puzzle.warehouse,newAction)
    return puzzle.warehouse.visualize()


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def addCoords(pos1,pos2):
    '''
    Adds 2 tuples together resulting in a new position
    pos1: tuple of form (x1,y1)
    pos2: tuple of form (x2,y2)
    return: tuple of form(x1+x2,y1+y2)
    '''
    return tuple(p+q for p,q in zip(pos1,pos2))

def tabooCells(puzzleFileName):
    '''
    This is a function called by the automatic marker.

    Your implementation should load a Sokoban puzzle from a text file,
    then identify the cells that should be avoided in the sense that if
    a box get pushed on such a cell then the puzzle becomes unsolvable.

    @param puzzleFileName: file name of the puzzle
         (same format as for the files in the warehouses directory)
    @return:
               A string representing the puzzle with the taboo cells marked with an 'X'.
               Apart from the 'X's, the string should follows the same format as the
               string returned by the method  Warehouse.visualize()
    '''
    myWare = cab320_sokoban.Warehouse()
    myWare.read_warehouse_file(puzzleFileName)
    tabooVis = list(myWare.visualize())
    tabooTup = tabooTuple(myWare.walls,myWare.targets)
    X,Y = zip(*myWare.walls)
    x_size, y_size = 1+max(X), 1+max(Y)
    for (x,y) in tabooTup:
        tabooVis[x+y*(x_size+1)] = "X"
    return "".join(tabooVis)

def tabooTuple(walls,targets):
    '''
    Builds a list of taboo cells from a given list of walls and targets using
    the static_taboo_corners and static_taboo_line functions to generate taboo
    cells.
    walls: tuple list of walls of form ((x1,y1),(x2,y2),..(xn,yn))
    targets: tuple list of targets of form ((x1,y1),(x2,y2),..(xn,yn))
    returns: list of tuples deemed as taboo of the form
    ((x1,y1),(x2,y2),..(xn,yn))
    '''
    tabooC = static_taboo_corners(walls,targets)
    tabooL = static_taboo_line(tabooC,walls,targets)
    taboo = cleanTaboo(tabooC+tabooL,targets,walls)
    #implement taboo line function and append that to taboo
    return taboo
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def static_taboo_corners(walls,targets):
    '''
    Computes the positions of taboo cells in a puzzle via the corner method
    If a wall is located in a configuration such that there is also a wall in
    a diagonal position, it marks the shared positions between the two as
    a taboo cell. If the corner cell is a target state, it does not set as taboo
    Examples denoted below where # is a wall, and X is taboo cell, . is goal
    ###X##########
    #XX#X   X#. .#
    #X          ##
    #############X
    inputs:
    walls: tuple list of (x,y) coordinates
    targets: tuple list of (x,y) coordinates
    returns:
    taboo: tuple list of (x,y) coordinates
    '''
    #analyze each cell and check diagonal cells
    taboo = []
    for (x,y) in walls:
        if (x+1,y+1) in walls:
            if(x+1,y) not in taboo:
                taboo.append(tuple((x+1,y)))
            if(x,y+1) not in taboo:
                taboo.append(tuple((x,y+1)))
        if (x+1,y-1) in walls:
            if(x+1,y) not in taboo:
                taboo.append(tuple((x+1,y)))
            if(x,y-1) not in taboo:
                taboo.append(tuple((x,y-1)))
        if (x-1,y+1) in walls:
            if(x-1,y) not in taboo:
                taboo.append(tuple((x-1,y)))
            if(x,y+1) not in taboo:
                taboo.append(tuple((x,y+1)))
        if (x-1,y-1) in walls:
            if(x-1,y) not in taboo:
                taboo.append(tuple((x-1,y)))
            if(x,y-1) not in taboo:
                taboo.append(tuple((x,y-1)))

    #remove any taboo cells that are also a wall
    removeTab = []
    for (x,y) in taboo:
        if (x,y) in walls:
            removeTab.append(tuple((x,y)))
        elif (x,y) in targets:
            removeTab.append(tuple((x,y)))
    for (x,y) in removeTab:
        taboo.remove(tuple((x,y)))
    return taboo

def static_taboo_line(taboo,walls,targets):
    '''
    Computes all the adjacent to wall taboo cells. Works by checking already
    existing taboo corner cells for any that are in line. Then checks the line
    of cells between those two. If at least one side of the checked line
    contains nothing but wall, those cells are marked as taboo.
    taboo: tuple list of form ((x1,y1),(x2,y2)...(xn,yn))
    walls: tuple list of form ((x1,y1),(x2,y2)...(xn,yn))
    returns: tuple list of form ((x1,y1),(x2,y2)...(xn,yn)) cells marked as
    taboo
    '''
    newTaboo = []
    for tab1 in taboo:
        for tab2 in taboo:
            if tab1 != tab2:
                #calculate for every pair of taboo cells
                line = isInLine(tab1,tab2) # check if they're in line
                if line is 'y': #if they share the y direction
                    distance = abs(tab1[1]-tab2[1])
                    y1,y2 = tab1[1],tab2[1]
                    if y1>y2: #swap values to make calculation easier
                        y1,y2 = y2,y1

                    xl = [tab1[0]-1 for a in range(distance)]
                    y = [b for b in range(y1,y2)]
                    xr = [tab1[0]+1 for a in range(distance)]

                    left = zip(xl,y)
                    right = zip(xr,y)
                    #check if either of the adjacent lines is pure wall
                    if checkAllIn(walls,left) or checkAllIn(walls,right):
                        x = [tab1[0] for a in range(distance)]
                        line = zip(x,y)
                        if checkTargetsInLine(targets,line):
                            newTaboo += zip(x,y)

                if line is 'x': #if they share the x direction
                    distance = abs(tab1[0]-tab2[0])
                    y1,y2 = tab1[0],tab2[0]
                    if y1>y2: #swap values to make calculation easier
                        y1,y2 = y2,y1

                    yu = [tab1[1]-1 for a in range(distance)]
                    x = [b for b in range(y1,y2)]
                    yd = [tab1[1]+1 for a in range(distance)]

                    up = zip(x,yu)
                    down = zip(x,yd)
                    #check if either of the adjacent lines is pure wall
                    if checkAllIn(walls,up) or checkAllIn(walls,down):
                        y = [tab1[1] for a in range(distance)]
                        line = zip(x,y)
                        if checkTargetsInLine(targets,line):
                            newTaboo += zip(x,y)


    return newTaboo

def cleanTaboo(taboo,targets,walls):
    clean = []
    newTaboo = list(set(taboo)) #removes multiples
    for pos in newTaboo:
        if pos in targets or pos in walls:
            clean.append(pos)
    for pos in clean:
        newTaboo.remove(pos)
    return newTaboo
def checkTargetsInLine(targets,line):
    for pos in line:
        if pos in targets:
            return False
    return True

def checkAllIn(walls,check):
    '''
    Checks all cells in line are in another set
    walls: list of tuples of form (x,y)
    check: list of tuples of form (x,y)
    return: False if all of check isn't in walls, True if it is
    '''
    for ch in check:
        if ch not in walls:
            return False
    return True

def isInLine(pos1,pos2):
    '''
    Checks 2 tuples and returns the grid direction they share
    pos1: tuple of form (x,y)
    pos2: tuple of form (x,y)
    return: 'x' if they share the x values, 'y' if they share the y values
    returns None if they share no values.
    '''
    if pos1[0] is pos2[0]:
        return 'y'
    elif pos1[1] is pos2[1]:
        return 'x'
    return None

def solveSokoban_elementary(puzzleFileName, timeLimit = None):
    '''
    This is a function called by the automatic marker.

    This function should solve the puzzle defined in a file.

    @param puzzleFileName: file name of the puzzle
         (same format as for the files in the warehouses directory)
    @param time_limit: The time limit for this agent in seconds .
    @return:
        A list of strings.
        If timeout return  ['Timeout']
        If puzzle cannot be solved return ['Impossible']
        If a solution was found, return a list of elementary actions that solves
            the given puzzle coded with 'Left', 'Right', 'Up', 'Down'
            For example, ['Left', 'Down', Down','Right', 'Up', 'Down']
            If the puzzle is already in a goal state, simply return []
    '''
    puzzle = SokobanPuzzle(puzzleFileName)
    if(puzzle.goal_test(puzzle.initial)):
        return []
    if timeLimit is None:
        try:
            sol = cab320_search.astar_search(puzzle,lambda n:puzzle.h(n))
        except AssertionError:
            return ['Impossible']
    timer = threading.Timer(timeLimit,thread.interrupt_main)
    try:
        timer.start()
        try:
            sol = cab320_search.astar_search(puzzle,lambda n:puzzle.h(n))
        except AssertionError:
            time.cancel()
            return ['Impossible']
        timer.cancel()
    except:
        timer.cancel()
        return ['Timeout']
    return puzzle.return_path(sol.path())


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

def solveSokoban_macro(puzzleFileName, timeLimit = None):
    '''
    This is a function called by the automatic marker.

    This function has the same purpose as 'solveSokoban_elementary', but
    it should internally use macro actions as suggested
    in the assignment description. Although it internally uses macro
    actions, this function should return a sequence of
    elementary  actions.


    @param puzzleFileName: file name of the puzzle
         (same format as for the files in the warehouses directory)
    @param time_limit: The time limit for this agent in seconds .
    @return:
        A list of strings.
        If timeout return  ['Timeout']
        If puzzle cannot be solved return ['Impossible']
        If a solution was found, return a list elementary actions that solves
            the given puzzle coded with 'Left', 'Right', 'Up', 'Down'
            For example, ['Left', 'Down', Down','Right', 'Up', 'Down']
            If the puzzle is already in a goal state, simply return []
    '''
    print "Warning: Macro solution does not currently arrive at the solution"
    puzzle = SokobanPuzzleMacro(puzzleFileName)
    if(puzzle.goal_test(puzzle.initial)):
        return []
    if timeLimit is None:
        try:
            sol = cab320_search.astar_search(puzzle,lambda n:puzzle.h(n))
        except AssertionError:
            return ['Impossible']
    timer = threading.Timer(timeLimit,thread.interrupt_main)
    try:
        timer.start()
        try:
            sol = cab320_search.astar_search(puzzle,lambda n:puzzle.h(n))
        except AssertionError:
            time.cancel()
            return ['Impossible']
    except:
        timer.cancel()
        return ['Timeout']
    timer.cancel()

    return puzzle.return_path(sol.path())

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def testPuzzle(filename):
    print "Working on",filename
    print "-----------------------------------------------------------------"
    puzzle = SokobanPuzzle(filename)
    t0 = time.time()
    sol = cab320_search.astar_search(puzzle,lambda n:puzzle.h(n))
    t_final = time.time() - t0
    print "Solver took ",t_final, ' seconds'
    puzzle.print_solution(sol)
def runSolver():
    skip = ["warehouse_141.txt",'warehouse_177.txt','warehouse_137.txt','warehouse_127.txt']
    for filename in os.listdir("/home/nathanjp/git/CAB320Sokoban/2016_sokoban_BB/warehouses"):
        if filename in skip:
            continue
        print "Working on",filename
        print "-----------------------------------------------------------------"
        fullFilename = "warehouses/"+filename
        testPuzzle(fullFilename)
def visualize(worker,walls,targets,taboo,boxes):
    '''
    Return a string representation of the warehouse
    '''
    ##        x_size = 1+max(x for x,y in self.walls)
    ##        y_size = 1+max(y for x,y in self.walls)
    X,Y = zip(*walls) # pythonic version of the above
    x_size, y_size = 1+max(X), 1+max(Y)

    vis = [[" "] * x_size for y in range(y_size)]
    for (x,y) in walls:
        vis[y][x] = "#"
    for (x,y) in targets:
        vis[y][x] = "."
    for (x,y) in taboo:
        vis[y][x] = "X"
    # if worker is on a target display a "!", otherwise a "@"
    # exploit the fact that Targets has been already processed
    if vis[worker[1]][worker[0]] == ".": # Note y is worker[1], x is worker[0]
        vis[worker[1]][worker[0]] = "!"
    else:
        vis[worker[1]][worker[0]] = "@"
    # if a box is on a target display a "*"
    # exploit the fact that Targets has been already processed
    for (x,y) in boxes:
        if vis[y][x] == ".": # if on target
            vis[y][x] = "*"
        else:
            vis[y][x] = "$"
    return "\n".join(["".join(line) for line in vis])
def test_taboo():
    filename = "warehouses/warehouse_03.txt"
    puzzle = SokobanPuzzle(filename)
    walls = puzzle.warehouse.walls
    targets = puzzle.warehouse.targets
    worker = puzzle.warehouse.worker
    boxes = puzzle.warehouse.boxes

    fullTab = tabooTuple(puzzle.warehouse.walls,puzzle.warehouse.targets)

    print visualize(worker,walls,targets,fullTab,boxes)
def test_elementary():
    filename = "warehouses/warehouse_01.txt"
    path = solveSokoban_elementary(filename)
    print path
def test_macro():
    filename = "warehouses/warehouse_01.txt"
    puzzle = SokobanPuzzleMacro(filename)
    sol = cab320_search.astar_search(puzzle,lambda n:puzzle.h(n))
    print puzzle.warehouse.visualize()
    puzzle.print_solution(sol)
if __name__ == "__main__":
    # runSolver()
    #test_elementary()
    #test_macro()
    #test_taboo()
    testPuzzle("warehouses/warehouse_43.txt")
