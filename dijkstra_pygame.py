import pygame
from math import sqrt

# Constants
WIDTH, HEIGHT = 600, 300
GRID_SIZE = 30
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
BLUE = (0, 0, 255)
BLACK = (0, 0, 0)

# Initialize Pygame
pygame.init()
win = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Dijkstra's Pathfinding Animation")
clock = pygame.time.Clock()

class Node:
    def __init__(self, position, g_cost=0, parent=None):
        self.position = position
        self.g_cost = g_cost
        self.parent = parent

    def __eq__(self, other):
        return self.position == other.position

    def __hash__(self):
        return hash(self.position)

def draw_grid(path):
    for x in range(0, WIDTH, GRID_SIZE):
        for y in range(0, HEIGHT, GRID_SIZE):
            rect = pygame.Rect(x, y, GRID_SIZE, GRID_SIZE)
            if (x // GRID_SIZE, y // GRID_SIZE) in path:
                pygame.draw.rect(win, BLUE, rect)
            pygame.draw.rect(win, WHITE, rect, 1)

def draw_start_goal(start, goal):
    start_rect = pygame.Rect(start[0] * GRID_SIZE, start[1] * GRID_SIZE, GRID_SIZE, GRID_SIZE)
    goal_rect = pygame.Rect(goal[0] * GRID_SIZE, goal[1] * GRID_SIZE, GRID_SIZE, GRID_SIZE)
    pygame.draw.rect(win, GREEN, start_rect)
    pygame.draw.rect(win, RED, goal_rect)

def get_successors(node):
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (1, -1), (-1, -1), (-1, 1)]
    cost = {d: sqrt(2) if d in directions[4:] else 1 for d in directions}
    successors = []
    for d in directions:
        next_pos = (node.position[0] + d[0], node.position[1] + d[1])
        if 0 <= next_pos[0] < WIDTH // GRID_SIZE and 0 <= next_pos[1] < HEIGHT // GRID_SIZE:
            movement_cost = cost[d]
            successors.append(Node(next_pos, node.g_cost + movement_cost, node))
    return successors

def dijkstra_search(start, goal):
    open_set = set()
    closed_set = set()
    start_node = Node(start)
    goal_node = Node(goal)
    open_set.add(start_node)
    
    while open_set:
        current_node = min(open_set, key=lambda n: n.g_cost)
        if current_node.position == goal_node.position:
            return reconstruct_path(current_node)
        open_set.remove(current_node)
        closed_set.add(current_node)
        
        for successor in get_successors(current_node):
            if successor in closed_set:
                continue
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
            yield current_node, open_set, closed_set

def reconstruct_path(node):
    path = []
    while node:
        path.append(node.position)
        node = node.parent
    return path

start_position = (0, 0)
goal_position = (19, 9)
generator = dijkstra_search(start_position, goal_position)

running = True
path = []

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    try:
        current_node, open_set, closed_set = next(generator)
        path.append(current_node.position)
    except StopIteration:
        pass

    win.fill(BLACK)
    draw_grid(path)
    draw_start_goal(start_position, goal_position)
    pygame.display.update()
    clock.tick(10)

pygame.quit()