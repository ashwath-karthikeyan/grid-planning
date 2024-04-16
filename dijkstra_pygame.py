import pygame
from math import sqrt

# Constants
WIDTH, HEIGHT = 1200, 800  # Larger display area
NODE_RADIUS = 10           # Smaller node radius for more nodes
NODE_DISTANCE = 40         # Smaller distance to fit more nodes
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
BLACK = (0, 0, 0)

# Initialize Pygame
pygame.init()
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Dijkstra's Pathfinding Animation")
clock = pygame.time.Clock()

class Node:
    def __init__(self, position, g_cost=float('inf'), parent=None):
        self.position = position
        self.g_cost = g_cost
        self.parent = parent
        self.neighbors = []

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

    def add_neighbor(self, neighbor):
        self.neighbors.append(neighbor)

def create_graph():
    nodes = {}
    for x in range(NODE_DISTANCE, WIDTH, NODE_DISTANCE):
        for y in range(NODE_DISTANCE, HEIGHT, NODE_DISTANCE):
            nodes[(x, y)] = Node((x, y))
    for node in nodes.values():
        x, y = node.position
        for dx, dy in [(-NODE_DISTANCE, 0), (NODE_DISTANCE, 0), (0, -NODE_DISTANCE), (0, NODE_DISTANCE)]:
            if (x + dx, y + dy) in nodes:
                node.add_neighbor(nodes[(x + dx, y + dy)])
    return nodes

def draw_nodes_and_edges(nodes, final_path, open_set, closed_set, start, goal):
    for node in nodes.values():
        color = BLUE if node in closed_set else WHITE if node in open_set else WHITE
        if node.position in final_path:
            color = YELLOW
        if node == start:
            color = GREEN
        if node == goal:
            color = RED
        # Draw nodes
        pygame.draw.circle(win, color, node.position, NODE_RADIUS)

def dijkstra_search(start, goal):
    open_set = set()
    closed_set = set()
    start.g_cost = 0
    open_set.add(start)

    while open_set:
        current_node = min(open_set, key=lambda n: n.g_cost)
        if current_node == goal:
            return reconstruct_path(current_node)
        open_set.remove(current_node)
        closed_set.add(current_node)

        for neighbor in current_node.neighbors:
            if neighbor in closed_set:
                continue
            temp_g_cost = current_node.g_cost + sqrt((neighbor.position[0] - current_node.position[0])**2 + (neighbor.position[1] - current_node.position[1])**2)
            if temp_g_cost < neighbor.g_cost:
                neighbor.g_cost = temp_g_cost
                neighbor.parent = current_node
                open_set.add(neighbor)
            yield current_node, open_set, closed_set, []

def reconstruct_path(node):
    path = []
    while node:
        path.append(node.position)
        node = node.parent
    return path[::-1]  # Reverse path

nodes = create_graph()
start_position = (NODE_DISTANCE, NODE_DISTANCE)
goal_position = (WIDTH - NODE_DISTANCE, HEIGHT - NODE_DISTANCE)
start_node = nodes[start_position]
goal_node = nodes[goal_position]
generator = dijkstra_search(start_node, goal_node)

running = True
final_path = []

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    if not final_path:
        try:
            current_node, open_set, closed_set, _ = next(generator)
        except StopIteration as e:
            final_path = e.value

    win.fill(BLACK)
    draw_nodes_and_edges(nodes, final_path, open_set, closed_set, start_node, goal_node)
    pygame.display.update()
    clock.tick(10)

pygame.quit()