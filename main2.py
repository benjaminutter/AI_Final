#TO RUN THIS CODE YOU MUST USE TERMINAL COMMANDS OF 'python main2.py <start_row> <start_col> <end_row> <end_col>'

import heapq
import sys
import numpy as np
import cv2

class RobotDeliverySystem:
    def __init__(self, matrix):
        self.matrix = matrix
        self.rows = len(matrix)
        self.cols = len(matrix[0])

    def print_matrix(self):
        for row in self.matrix:
            print(' '.join(map(str, row)))

    def get_start_and_destination_positions(self):
        if len(sys.argv) != 5:
            print("Usage: python script.py <start_row> <start_col> <destination_row> <destination_col>")
            sys.exit(1)

        try:
            start_row = int(sys.argv[1])
            start_col = int(sys.argv[2])
            destination_row = int(sys.argv[3])
            destination_col = int(sys.argv[4])
            if not (0 <= start_row < self.rows and 0 <= start_col < self.cols
                    and 0 <= destination_row < self.rows and 0 <= destination_col < self.cols):
                raise ValueError("Input out of bounds.")
        except ValueError:
            print("Invalid input. Please enter valid integer coordinates within the bounds of the matrix.")
            sys.exit(1)

        return (start_row, start_col), (destination_row, destination_col)

    def dijkstra(self, start, destination):
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        visited = set()
        distance = {start: 0}
        parent = {}

        pq = [(0, start)]

        while pq:
            dist, node = heapq.heappop(pq)

            if node == destination:
                path = []
                while node in parent:
                    path.append(node)
                    node = parent[node]
                path.append(start)
                return list(reversed(path))

            visited.add(node)

            for dx, dy in directions:
                x, y = node[0] + dx, node[1] + dy
                if 0 <= x < self.rows and 0 <= y < self.cols and self.matrix[x][y] == 1:
                    if (x, y) not in visited:
                        new_distance = dist + 1
                        if new_distance < distance.get((x, y), float('inf')):
                            distance[(x, y)] = new_distance
                            parent[(x, y)] = node
                            heapq.heappush(pq, (new_distance, (x, y)))

        return None  # No path found

    def astar(self, start, destination):
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        def heuristic(node):
            return abs(node[0] - destination[0]) + abs(node[1] - destination[1])

        visited = set()
        g_score = {start: 0}
        f_score = {start: heuristic(start)}
        parent = {}

        pq = [(f_score[start], start)]

        while pq:
            _, node = heapq.heappop(pq)

            if node == destination:
                path = []
                while node in parent:
                    path.append(node)
                    node = parent[node]
                path.append(start)
                return list(reversed(path))

            visited.add(node)

            for dx, dy in directions:
                x, y = node[0] + dx, node[1] + dy
                if 0 <= x < self.rows and 0 <= y < self.cols and self.matrix[x][y] == 1:
                    if (x, y) not in visited:
                        tentative_g_score = g_score[node] + 1
                        if tentative_g_score < g_score.get((x, y), float('inf')):
                            parent[(x, y)] = node
                            g_score[(x, y)] = tentative_g_score
                            f_score[(x, y)] = tentative_g_score + heuristic((x, y))
                            heapq.heappush(pq, (f_score[(x, y)], (x, y)))

        return None  # No path found

    def mark_route_on_matrix(self, route, symbol):
        for row, col in route:
            self.matrix[row][col] = symbol

# Load the matrix from the text file
def load_matrix_from_file(filename):
    matrix = np.loadtxt(filename, dtype=int)
    return matrix

# Example usage:
image = cv2.imread('floorplan1_nolegend.jpg')

if image is None:
    print("Error: Unable to load image.")
    exit(1)

resized_image = cv2.resize(image, (50, 50))
gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
_, binary = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY)
binary = cv2.bitwise_not(binary)
matrix = (binary / 255).astype(int)

delivery_system = RobotDeliverySystem(matrix)

print("Initial Matrix:")
delivery_system.print_matrix()

start_pos, destination_pos = delivery_system.get_start_and_destination_positions()
print("Start position:", start_pos)
print("Destination position:", destination_pos)

# Calculate routes using Dijkstra's algorithm and A* algorithm
dijkstra_route = delivery_system.dijkstra(start_pos, destination_pos)
astar_route = delivery_system.astar(start_pos, destination_pos)

# Choose the fastest route
if dijkstra_route is None and astar_route is None:
    print("No valid routes found.")
elif dijkstra_route is None:
    print("Route using A* algorithm:", astar_route)
    delivery_system.mark_route_on_matrix(astar_route, 'A')
elif astar_route is None:
    print("Route using Dijkstra's algorithm:", dijkstra_route)
    delivery_system.mark_route_on_matrix(dijkstra_route, 'D')
else:
    if len(dijkstra_route) <= len(astar_route):
        print("Route using Dijkstra's algorithm (shorter or equal):", dijkstra_route)
        delivery_system.mark_route_on_matrix(dijkstra_route, 'D')
    else:
        print("Route using A* algorithm (shorter):", astar_route)
        delivery_system.mark_route_on_matrix(astar_route, 'A')

print("\nMatrix with routes marked:")
delivery_system.print_matrix()
