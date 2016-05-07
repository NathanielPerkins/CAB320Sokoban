import cab320_search

import cab320_sokoban


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
        self.walls = warehouse.walls
        self.worker = warehouse.worker
        self.boxes = warehouse.boxes
        self.targets = warehouse.targets

        X,Y = zip(*self.warehouse.walls)
        self.x_size = 1+max(X)
        self.y_size = 1+max(Y)

        self.taboo = tabooTup()
        self.initial = (tuple(self.worker),tuple(self.boxes))

    '''
    def __init__(self, puzzleFileName):
        self.myWare = cab320_sokoban.Warehouse()
        self.myWare.read_warehouse_file(puzzleFileName)
        self.walls = myWare.walls
        self.targets = myWare.targets
        self.worker = myWare.worker
        self.boxes = myWare.boxes
        self.taboo = []
        self.static_taboo_corners()
        # get taboo cells
    '''
    def goal_test(self,state):
        """Return True if the state is a goal. The default method compares the
        state to self.goal, as specified in the constructor. Override this
        method if checking against a single self.goal is not enough."""
        return set(self.targets)==set(state[1])

    def actions(self, state):
        """
        Return the actions that can be executed in the given
        state.
        """
        emptyCells = []
        worker = state[0]
        boxes = state[1]
        walls = self.warehouse.walls
        taboo = self.warehouse.taboo

        directions = {(0, -1):'Up',(0, 1):'Down',(-1, 0):'Left',(1, 0):'Right'}

        for coords in directions.keys():
                newCoord = addTuples(coords,worker)
                if(newCoord not in walls and newCoord not in boxes):
                        emptyCells.append(directions.get(coords))
                elif newCoord in boxes:
                        extendedCoord = addTuples(coords,coords)
                        extendedNewCoord = addTuples(extendedCoord,worker)
                        if (extendedNewCoord not in boxes) and (extendedNewCoord not in walls) and (extendedNewCoord not in taboo):
                                emptyCells.append(directions.get(coords))


        return emptyCells

    def result(self, state, action):
        """Return the state that results from executing the given
        action in the given state. The action must be one of
        self.actions(state).
        """
        assert action in self.actions(state)
        worker = state[0]
        boxes = list(state[1])

        coords = {(0, -1):'Up',(0, 1):'Down',(-1, 0):'Left',(1, 0):'Right'}
        move = coords.get(action)
        new_pos = addTuples(worker,move)
        if new_pos in boxes:
                #move box
                box_index = boxes.index(new_pos)
                boxes[box_index] = addTuples(new_pos,move)
                #move player

        worker = new_pos

        return (worker,tuple(boxes))

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

        # path is list of nodes from initial state (root of the tree)
        # to the goal_node
        path = goal_node.path()
        # print the solution
        print "Solution takes {0} steps from the initial state".format(len(path)-1)
        self.worker = path[0].state[0]
        self.boxes = path[0].state[1]
        print self.warehouse.visualize()
        print "to the goal state"
        self.worker = path[-1].state[0]
        self.boxes = path[-1].state[1]
        print self.warehouse.visualize()
        print "Below is the sequence of moves\n"
        actionSequence = []
        for node in path:
            if node.action is not None:
                actionSequence.append(node.action)
        print actionSequence

    def h(self,node):
        h = 0
        worker = node.state[0]
        boxes = node.state[1]
        #targets = self.warehouse.targets
        for box in boxes:
            closestGoal = closestGoal(box,self.targets)
            h = h + manhattanDistance(box,closestGoal)
        return cost


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def closestGoal(box,targets):
    distance = []
    for target in targets:
        distance.append(manhattanDistance(box,target))
    return min(distance)

def manhattanDistance(p1,p2):
    return math.fabs((p1[0]-p2[0])) + math.fabs((p1[1]-p2[1]))

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

    p = SokobanPuzzle(puzzleFileName)
    for action in actionSequence:
            new_action = action[0]
            if new_action not in p.actions(p.warehouse):
                    return 'Failure'
            p.warehouse = p.result(p.warehouse,new_action)

    return p.warehouse.visualize()


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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
    taboo = static_taboo_corners(walls,targets)
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
    print taboo
    removeTab = []
    for (x,y) in taboo:
        if (x,y) in walls:
            removeTab.append(tuple((x,y)))
        elif (x,y) in targets:
            removeTab.append(tuple((x,y)))
    for (x,y) in removeTab:
        taboo.remove(tuple((x,y)))
    return taboo

def static_taboo_line(taboo,walls):
    taboo = []
    for tab1 in taboo:
        for tab2 in taboo if tab1 is not tab2:
            line = isInLine()
            if line is 'y':
                if checkFreedom(tab1,tab2):

def checkFreedom(pos1,pos2,direction):
    side1 = 0
    side2 = 0
    if direction is 'y'
        if pos1[0] > pos2[0]:
            pos1,pos2 = pos2,pos1
        for i in range(pos1[0]+1 pos2[0]):
            if ()

def isInLine(pos1,pos2):
    if pos1[0] is pos2[0]:
        return 'x'
    elif pos1[1] is pos2[1]:
        return 'y'
    return ''

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

##         "INSERT YOUR CODE HERE"

    raise NotImplementedError()

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

##         "INSERT YOUR CODE HERE"

    raise NotImplementedError()

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
def test():
        import os, re
        startAt = ['warehouse_02.txt']
        skip = [] #skip = ['warehouse_5.txt', 'warehouse_7.txt', 'warehouse_33.txt', 'warehouse_35.txt', 'warehouse_59.txt']
        warehouseFiles = [f for f in os.listdir('warehouses/')]
        warehouseFiles.sort(key=lambda var:[int(x) if x.isdigit() else x for x in re.findall(r'[^0-9]|[0-9]+', var)])

        if len(startAt) is 1:
                index = warehouseFiles.index(startAt[0])
                if index is not None:
                        warehouseFiles = warehouseFiles[index:]

        for level in warehouseFiles:
                if level in skip:
                        continue
                p = SokobanPuzzle('warehouses/'+level)
                print '------------------------------------------------------'
                print 'Running solver on '+level+'...'
                #print(p.warehouse.visualize())
                t0 = time.time()

                #sol_ts = breadth_first_tree_search(p)
                sol_ts = astar_search(p,lambda n:p.heuristic(n))

                t1 = time.time()

                p.print_solution_simple(sol_ts)
                print "Solver took ",t1-t0, ' seconds'


if __name__ == "__main__":
    #test()
    filename = 'warehouses/warehouse_03.txt'
    myWare = cab320_sokoban.Warehouse()
    myWare.read_warehouse_file(filename)
    walls = myWare.walls
    targets = myWare.targets
    taboo = tabooCells(filename)
    print myWare.visualize()
    print taboo
