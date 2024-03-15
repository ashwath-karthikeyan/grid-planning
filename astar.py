from matplotlib import pyplot as plt


# Define constants
MAX_ANGULAR_VELOCITY = 45
VELOCITY = 5
TURN_COST = MAX_ANGULAR_VELOCITY / 8  # Cost for 45-degree turn

# Node representation
class Node:
    def __init__(self, position, g_cost=0, parent=None):
        self.position = position
        self.g_cost = g_cost  # Total cost from start node to this node
        self.parent = parent  # Parent node in path

# Helper function to calculate turn cost
def calculate_turn_cost(parent, current, next_node):
    if parent is None:
        return 0
    direction_to_current = tuple(c - p for c, p in zip(current.position, parent.position))
    direction_to_next = tuple(n - c for n, c in zip(next_node, current.position))
    if direction_to_current == direction_to_next:
        return 0  # No turn cost if straight
    else:
        return TURN_COST  # Turn cost for 45-degree turn

# Generate successors for 8-connected grid
def get_successors(node):
    directions = [(0, 1), (1, 1), (1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1)]
    successors = []
    for d in directions:
        next_position = (node.position[0] + d[0], node.position[1] + d[1])
        turn_cost = calculate_turn_cost(node.parent, node, next_position)
        next_node = Node(next_position, node.g_cost + 1 + turn_cost, node)
        successors.append(next_node)
    return successors

# Heuristic function for A* (Manhattan distance)
def heuristic(node, goal):
    return abs(node.position[0] - goal[0]) + abs(node.position[1] - goal[1])

# A* search algorithm with turn penalty
def a_star_search(start, goal):
    open_set = set()
    closed_set = set()
    start_node = Node(start)
    goal_node = Node(goal)
    
    open_set.add(start_node)
    
    while open_set:
        current_node = min(open_set, key=lambda n: n.g_cost + heuristic(n, goal))
        
        if current_node.position == goal_node.position:
            return reconstruct_path(current_node)
        
        open_set.remove(current_node)
        closed_set.add(current_node)
        
        for successor in get_successors(current_node):
            if successor in closed_set:
                continue
            if successor not in open_set:
                open_set.add(successor)
            else:
                # Check if we've found a better path
                existing_node = open_set.pop(successor)
                if successor.g_cost < existing_node.g_cost:
                    open_set.add(successor)
                else:
                    open_set.add(existing_node)
    
    return []  # Return empty path if goal not found

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
path = a_star_search(start_position, goal_position)

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