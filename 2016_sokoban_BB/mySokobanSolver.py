import cab320_search

import cab320_sokoban


# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

class SokobanPuzzle(Problem):
        '''
        Class to represent a Sokoban puzzle.
        Your implementation should be compatible with the
        search functions of the module  cab320_search

        '''
##         "INSERT YOUR CODE HERE"

    def __init__(self, puzzleFileName):
        raise NotImplementedError()

        
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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

##         "INSERT YOUR CODE HERE"

        raise NotImplementedError()

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

##         "INSERT YOUR CODE HERE"

        raise NotImplementedError()

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

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

