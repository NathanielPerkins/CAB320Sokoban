#### IMPORTANT: THIS FILE IS NOT USED IN MARKING !!!


from cab320_sokoban import Warehouse
from mySokobanSolver import *

if __name__ == "__main__":
  problem_file = "./warehouses/warehouse_1.txt"

  """
  Display problem
  """
  field = Warehouse()
  field.read_warehouse_file(problem_file)
  print("\nProblem:")
  print field.visualize()

  """
  Check Actions
  """
  #test case 1
  actionSequence = ['Left', 'Down','Down','Right', 'Up', 'Down']
  result = checkActions(problem_file,actionSequence)
  print("\nAttempting action sequence:\n%s"%(actionSequence))
  print("Result:\n%s"%(result))
  #test case 2
  actionSequence = ['Down','Left','Up','Right','Right', 'Right','Down','Left', 'Up', 'Left','Left','Down','Down','Right','Up','Left','Up','Right','Up','Up','Left','Down']
  result = checkActions(problem_file,actionSequence)
  print("\nAttempting action sequence:\n%s"%(actionSequence))
  print("Result:\n%s"%(result))
  """
  Show Taboo Cells
  """
  result = tabooCells(problem_file)
  print("\nTaboo Cells:")
  print(result)
  """
  Solve problem
  """
  timeout = 600 #Extend timeout if difficult e.g. warehouse_59
  result = solveSokoban(problem_file,timeout)
  print("\nSolution:")
  print(result)
