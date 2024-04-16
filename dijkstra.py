from matplotlib import pyplot as plt
from math import pi, sqrt

# Define constants
MAX_ANGULAR_VELOCITY = 45
VELOCITY = 5

# Node representation
class Node:
    def __init__(self, position, g_cost=0, parent=None):
        self.position = position
        self.g_cost = g_cost  # Total cost from start node to this node
        self.parent = parent  # Parent node in path

# Generate successors for 8-connected grid
def get_successors(node):
    # Define directions with their corresponding movement costs
    cardinal_directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
    intercardinal_directions = [(1, 1), (1, -1), (-1, -1), (-1, 1)]
    cardinal_cost = 1  # Cost for cardinal movements
    intercardinal_cost = sqrt(2)  # Cost for intercardinal (diagonal) movements
    
    successors = []
    for d in cardinal_directions + intercardinal_directions:
        next_position = (node.position[0] + d[0], node.position[1] + d[1])
        
        # Determine if the movement is cardinal or intercardinal and assign the correct cost
        if d in cardinal_directions:
            movement_cost = cardinal_cost
        else:
            movement_cost = intercardinal_cost
        
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

# Reconstruct path from goal to start
def reconstruct_path(node):
    path = []
    while node:
        path.append(node.position)
        node = node.parent
    return path[::-1]  # Return reversed path

# Example usage
start_position = (0, 0)  # Starting coordinates
goal_position = (20, 9)  # Goal coordinates
# path = a_star_search(start_position, goal_position)
path = dijkstra_search(start_position,goal_position)

print("Path found:", path)

x_coords, y_coords = zip(*path)

# Plotting
plt.figure(figsize=(10, 10))  # Set the figure size for better visibility
plt.plot(x_coords, y_coords, '-o', label='Path')  # Plot path as line with dots at nodes
plt.scatter([start_position[0]], [start_position[1]], color='green', label='Start')  # Start position
plt.scatter([goal_position[0]], [goal_position[1]], color='red', label='Goal')  # Goal position
plt.title("Dijkstra's Pathfinding with Turn Penalty")
plt.xlabel('X Coordinate')
plt.ylabel('Y Coordinate')
plt.legend()
plt.grid(True)
plt.show()