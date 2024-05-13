import pygame
from math import sqrt, atan2, pi

# Constants
WIDTH, HEIGHT = 1200, 800
NODE_RADIUS = 10
NODE_DISTANCE = 40

# PyGame stuff
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)
OBSTACLE_COLOR = (128, 128, 128)

# PyGame setup
pygame.init()
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Dijkstra's Pathfinding Animation with Obstacles")
clock = pygame.time.Clock()
font = pygame.font.Font(None, 30)  # Create a font object

def draw_nodes_and_edges(nodes, final_path, open_set, closed_set, start, goal, iteration_count, path_length):
    win.fill(BLACK)
    for ox, oy, ow, oh in obstacles:
        pygame.draw.rect(win, OBSTACLE_COLOR, (ox, oy, ow, oh))
    for node in nodes.values():
        color = YELLOW if node.position in final_path else BLUE if node in closed_set else WHITE if node in open_set else WHITE
        if node == start:
            color = GREEN
        if node == goal:
            color = RED
        pygame.draw.circle(win, color, node.position, NODE_RADIUS)
    
    # Render iteration count and path length in the top right corner
    iterations_surf = font.render(f"Iterations: {iteration_count}", True, WHITE)
    path_length_surf = font.render(f"Path Length: {path_length}", True, WHITE)
    iterations_pos = (WIDTH - iterations_surf.get_width() - 200, 5)  # Top right, adjust margin
    path_length_pos = (WIDTH - path_length_surf.get_width() - 30, 5)  # Below the iterations count
    
    win.blit(iterations_surf, iterations_pos)
    win.blit(path_length_surf, path_length_pos)

# Planner setup
class Node:
    def __init__(self, position):
        self.position = position
        self.g_cost = float('inf')
        self.parent = None
        self.neighbors = []
        self.direction = None

    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

def is_within_obstacle(x, y, obstacles):
    for ox, oy, ow, oh in obstacles:
        if ox <= x <= ox + ow and oy <= y <= oy + oh:
            return True
    return False

def create_graph(obstacles):
    nodes = {}
    for x in range(NODE_DISTANCE, WIDTH, NODE_DISTANCE):
        for y in range(NODE_DISTANCE, HEIGHT, NODE_DISTANCE):
            if not is_within_obstacle(x, y, obstacles):
                nodes[(x, y)] = Node((x, y))
    for node in nodes.values():
        x, y = node.position
        four_connected = [
            (-NODE_DISTANCE, 0), (NODE_DISTANCE, 0), (0, -NODE_DISTANCE), (0, NODE_DISTANCE)
            ]
        eight_connected = [
            (-NODE_DISTANCE, 0), (NODE_DISTANCE, 0),
            (0, -NODE_DISTANCE), (0, NODE_DISTANCE),
            (-NODE_DISTANCE, -NODE_DISTANCE), (NODE_DISTANCE, NODE_DISTANCE),
            (NODE_DISTANCE, -NODE_DISTANCE), (-NODE_DISTANCE, NODE_DISTANCE)  # Diagonal connections
        ]
        for dx, dy in eight_connected:
            neighbor_position = (x + dx, y + dy)
            if neighbor_position in nodes:
                node.add_neighbor(nodes[neighbor_position])
    return nodes

def calculate_direction(from_node, to_node):
    return atan2(to_node.position[1] - from_node.position[1], to_node.position[0] - from_node.position[0])

# Planner algorithm
def gradient(start, goal):
    open_set = set([start])
    closed_set = set()
    current = start
    current.g_cost = 0

    while open_set:
        current = min(open_set, key=lambda o: o.g_cost)
        if current == goal:
            return reconstruct_path(current)
        
        open_set.remove(current)
        closed_set.add(current)

        for neighbor in current.neighbors:
            if neighbor in closed_set:
                continue
            
            tentative_g_cost = current.g_cost + heuristic(neighbor, goal)
            
            if neighbor not in open_set:
                open_set.add(neighbor)
            elif tentative_g_cost >= neighbor.g_cost:
                continue
            
            neighbor.g_cost = tentative_g_cost
            neighbor.parent = current

        yield current, open_set, closed_set, None
    
    raise StopIteration(reconstruct_path(goal))

def heuristic(node, goal):
    #0 for euclidean, 1 for manhattan, 2 for octile
    norm = 1
    if norm == 1:
        return sqrt((node.position[0] - goal.position[0]) ** 2 + (node.position[1] - goal.position[1]) ** 2)

    elif norm == 1:
        return abs(node.position[0] - goal.position[0]) + abs(node.position[1] - goal.position[1])
    
    elif norm == 2:
        dx = abs(node.position[0] - goal.position[0])
        dy = abs(node.position[1] - goal.position[1])
        return max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)

def reconstruct_path(node):
    path = []
    while node:
        path.append(node.position)
        node = node.parent
    return path[::-1]  # Reverse path

# Obstacle definitions (top-left corner x, top-left corner y, width, height)
obstacles = [
    (400, 200, 200, 400), (800, 600, 200, 100)
]

nodes = create_graph(obstacles)
start_position = (NODE_DISTANCE, NODE_DISTANCE)
goal_position = (WIDTH - NODE_DISTANCE, HEIGHT - NODE_DISTANCE)
start_node = nodes[start_position]
goal_node = nodes[goal_position]
generator = gradient(start_node, goal_node)

running = True
final_path = []
iteration_count = 0
path_length = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    if not final_path:
        try:
            current_node, open_set, closed_set, _ = next(generator)
            iteration_count += 1
        except StopIteration as e:
            final_path = e.value
            path_length = len(final_path)
    draw_nodes_and_edges(nodes, final_path, open_set, closed_set, start_node, goal_node, iteration_count, path_length)
    pygame.display.update()
    clock.tick(100)

pygame.quit()