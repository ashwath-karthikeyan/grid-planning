# Define obstacles and bounds
# OBSTACLES = [sim.obstacle_list]
# BOUNDS = (0, 0, sim.width, sim.height)
# start_position = (sim.robots[0])  # Starting coordinates
# goal_position = (sim.robotGoals[0])  # Goal coordinates

import matplotlib.pyplot as plt
import math

# Define constants
VELOCITY = 5

OBSTACLES = [(0,5), (2,4), (7,10)]
BOUNDS = (0, 0, 20, 11) # Example bounds
start_position = (5,0)  # Example Starting coordinates
goal_position = (0,0)  # Example Goal coordinates

# Node representation
class Node:
    def __init__(self, position, g_cost=0, parent=None):
        self.position = position
        self.g_cost = g_cost  # Total cost from start node to this node
        self.parent = parent  # Parent node in path

def is_position_valid(position):
    """Check if the position is within bounds and not an obstacle."""
    x, y = position
    if x < BOUNDS[0] or x > BOUNDS[2] or y < BOUNDS[1] or y > BOUNDS[3]:
        return False  # Out of bounds
    if position in OBSTACLES:
        return False  # Position is an obstacle
    return True

def get_successors(node):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    movement_cost = 1
    
    successors = []
    for d in directions:
        next_position = (node.position[0] + d[0], node.position[1] + d[1])
        if is_position_valid(next_position):
            next_node = Node(next_position, node.g_cost + movement_cost, node)
            successors.append(next_node)
    return successors

def dijkstra_search(start, goal):
    open_set = set()
    closed_set = set()
    start_node = Node(start)
    goal_node = Node(goal)
    
    open_set.add(start_node)
    
    while open_set:
        # Select node with minimum g_cost in open set
        current_node = min(open_set, key=lambda n: n.g_cost)

        if current_node.position == goal_node.position:
            return reconstruct_path(current_node)
        
        open_set.remove(current_node)
        closed_set.add(current_node)
        
        for successor in get_successors(current_node):
            if successor in closed_set:
                continue
            # Improved handling for open_set updates
            in_open_set = False
            for open_node in open_set:
                if open_node.position == successor.position:
                    in_open_set = True
                    if successor.g_cost < open_node.g_cost:
                        open_set.remove(open_node)
                        open_set.add(successor)
                    break
            if not in_open_set:
                open_set.add(successor)

    return []  # Return empty path if goal not found

# Reconstruct path from goal to start
def reconstruct_path(node):
    path = []
    while node:
        path.append(node.position)
        node = node.parent
    return path[::-1]  # Return reversed path

# Assuming multiple start and goal positions for different robots
start_positions = [(5,0), (1,2)]  # Example start positions for multiple robots
goal_positions = [(0,0), (6,10)]  # Example goal positions for multiple robots

# The rest of your definitions (Node class, is_position_valid, get_successors, dijkstra_search, reconstruct_path) remain the same

def find_paths_for_all_robots(start_positions, goal_positions):
    all_paths = []
    for start_position, goal_position in zip(start_positions, goal_positions):
        path = dijkstra_search(start_position, goal_position)
        all_paths.append(path)
    return all_paths

all_paths = find_paths_for_all_robots(start_positions, goal_positions)

# Now, to plot the paths for all robots
plt.figure(figsize=(10, 10))
for index, path in enumerate(all_paths):
    if path:  # Check if the path is not empty
        x_coords, y_coords = zip(*path)
        plt.plot(x_coords, y_coords, '-o', label=f'Path for Robot {index + 1}')
    
# Plot start and goal positions for each robot
for index, (start_position, goal_position) in enumerate(zip(start_positions, goal_positions)):
    plt.scatter([start_position[0]], [start_position[1]], label=f'Start {index + 1}')
    plt.scatter([goal_position[0]], [goal_position[1]], label=f'Goal {index + 1}')

# Plot obstacles
for obstacle in OBSTACLES:
    plt.scatter([obstacle[0]], [obstacle[1]], color='black', label='Obstacle' if 'Obstacle' not in plt.gca().get_legend_handles_labels()[1] else "")

plt.xlim([BOUNDS[0], BOUNDS[2]])
plt.ylim([BOUNDS[1], BOUNDS[3]])
plt.grid(True)
plt.legend()
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.title('Pathfinding for Multiple Robots with Obstacle Avoidance and Bounds')

plt.show()