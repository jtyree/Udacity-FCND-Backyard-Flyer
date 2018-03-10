import astar
import numpy as np

# Define a start and goal location
start = (0, 0)
goal = (4, 4)
# Define your grid-based state space of obstacles and free space
grid = np.array([
    [0, 1, 0, 0, 0, 0],
    [0, 1, 0, 0, 1, 0],
    [0, 1, 0, 1, 0, 0],
    [0, 0, 0, 1, 1, 0],
    [0, 0, 0, 1, 0, 0],
])

path, path_cost = astar.a_star(grid, astar.heuristic, start, goal)
print(path_cost, path)

# S -> start, G -> goal, O -> obstacle
s = astar.visualize_path(grid, path, start)
print(s)
