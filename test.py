import heapq

# Define grid base
GRID_BASE = [[1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
             [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
             [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1],
             [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
             [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]]

# Define node class
class Node:
    def __init__(self, x, y, value):
        self.x = x
        self.y = y
        self.value = value
        self.parent = None
        self.g_score = float('inf')
        self.f_score = float('inf')

    def __lt__(self, other):
        return self.f_score < other.f_score

# A* algorithm
def astar(start, goal, grid):
    open_list = []
    closed_set = set()

    start_node = Node(start[0], start[1], grid[start[0]][start[1]])
    start_node.g_score = 0
    start_node.f_score = heuristic(start, goal)

    heapq.heappush(open_list, start_node)

    while open_list:
        current_node = heapq.heappop(open_list)

        if current_node.x == goal[0] and current_node.y == goal[1]:
            path = reconstruct_path(current_node)
            return path

        closed_set.add((current_node.x, current_node.y))

        neighbors = get_neighbors(current_node, grid)
        for neighbor in neighbors:
            if (neighbor.x, neighbor.y) in closed_set:
                continue

            tentative_g_score = current_node.g_score + 1

            if tentative_g_score < neighbor.g_score:
                neighbor.parent = current_node
                neighbor.g_score = tentative_g_score
                neighbor.f_score = neighbor.g_score + heuristic((neighbor.x, neighbor.y), goal)

                if neighbor not in open_list:
                    heapq.heappush(open_list, neighbor)

    return None

# Heuristic function (Euclidean distance)
def heuristic(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

# Get neighboring nodes
def get_neighbors(node, grid):
    neighbors = []
    directions = [(0, -1), (0, 1), (-1, 0), (1, 0)]  # Up, Down, Left, Right

    for direction in directions:
        x = node.x + direction[0]
        y = node.y + direction[1]

        if 0 <= x < len(grid) and 0 <= y < len(grid[0]) and grid[x][y] == 0:
            neighbor = Node(x, y, grid[x][y])
            neighbors.append(neighbor)

    return neighbors

# Reconstruct path from goal to start
def reconstruct_path(node):
    path = []
    while node.parent:
        path.append((node.x, node.y))
        node = node.parent
    path.append((node.x, node.y))
    path.reverse()
    return path

# Test the A* algorithm
start_pos = (1, 1)
goal_pos = (11, 11)
path = astar(start_pos, goal_pos, GRID_BASE)
print("Path:", path)