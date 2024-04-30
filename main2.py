#TO RUN THIS CODE YOU MUST USE TERMINAL COMMANDS OF 'python main2.py <start_row> <start_col> <end_row> <end_col>'

import heapq
import sys
import numpy as np
import cv2
import tkinter as tk
import tkinter.messagebox as messagebox

class PositionInputPopup:
    def __init__(self, master):
        self.master = master
        self.master.title("Position Input")
        
        self.label_start = tk.Label(master, text="Start Position (row, col):")
        self.label_start.grid(row=0, column=0)
        self.entry_start = tk.Entry(master)
        self.entry_start.grid(row=0, column=1)
        
        self.label_destinations = []
        self.entry_destinations = []
        for i in range(4):
            label = tk.Label(master, text=f"Destination {i+1} (row, col):")
            label.grid(row=i+1, column=0)
            entry = tk.Entry(master)
            entry.grid(row=i+1, column=1)
            self.label_destinations.append(label)
            self.entry_destinations.append(entry)
        
        self.button_submit = tk.Button(master, text="Submit", command=self.submit)
        self.button_submit.grid(row=5, columnspan=2)
        
    def submit(self):
        start_pos_str = self.entry_start.get()
        destination_pos_str = [entry.get() for entry in self.entry_destinations]
    
        # Convert start and end positions to tuples of integers
        try:
            start_pos = tuple(map(int, start_pos_str.split(',')))
        except ValueError:
            print("Invalid input. Please enter valid integer coordinates in the format 'row, col'.")
            return

        
        destination_positions = []
        for pos_str in destination_pos_str:
            if pos_str.strip():  # Check if the string is not empty or only whitespace
                try:
                    destination_positions.append(tuple(map(int, pos_str.split(','))))
                except ValueError:
                    print(f"Invalid input for destination position '{pos_str}'. Please enter valid integer coordinates in the format 'row, col'.")
                    return

        if not destination_positions:
            print("No destination positions provided.")
            return

        # Calculate routes and mark them on the matrix for each destination
        for destination in destination_positions:
            # Calculate routes using Dijkstra's algorithm and A* algorithm
            dijkstra_route = delivery_system.dijkstra(start_pos, destination)
            astar_route = delivery_system.astar(start_pos, destination)


            # Choose the fastest route
            if dijkstra_route is None and astar_route is None:
                print("No valid routes found.")
            elif dijkstra_route is None:
                print("Route using A* algorithm:", astar_route)
                delivery_system.mark_route_on_matrix(astar_route, 9)
            elif astar_route is None:
                print("Route using Dijkstra's algorithm:", dijkstra_route)
                delivery_system.mark_route_on_matrix(dijkstra_route, 8)
            else:
                if len(dijkstra_route) <= len(astar_route):
                    print("Route using Dijkstra's algorithm (shorter or equal):", dijkstra_route)
                    delivery_system.mark_route_on_matrix(dijkstra_route, 8)
                else:
                    print("Route using A* algorithm (shorter):", astar_route)
                    delivery_system.mark_route_on_matrix(astar_route, 9)

        print("\nMatrix with routes marked:")
        delivery_system.print_matrix()
        self.master.destroy()


def open_position_input_popup():
    root = tk.Tk()
    popup = PositionInputPopup(root)
    root.mainloop()

class RobotDeliverySystem:
    def __init__(self, matrix):
        self.matrix = matrix
        self.rows = len(matrix)
        self.cols = len(matrix[0])

    def print_matrix(self):
        for row in self.matrix:
            print(' '.join(map(str, row)))

    def get_start_and_destination_positions(self, start_pos, end_pos):
        try:
            start_row, start_col = map(int, start_pos.split(','))
            end_row, end_col = map(int, end_pos.split(','))
            if not (0 <= start_row < self.rows and 0 <= start_col < self.cols
                    and 0 <= end_row < self.rows and 0 <= end_col < self.cols):
                raise ValueError("Input out of bounds.")
        except ValueError:
            print("Invalid input. Please enter valid integer coordinates within the bounds of the matrix.")
            sys.exit(1)

        return (start_row, start_col), (end_row, end_col)

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
        print("Marking route on matrix with symbol:", symbol)
        print("Route to mark:", route)
        for row, col in route:
            print("Marking symbol at position:", row, col)
            self.matrix[row][col] = symbol
        print("Matrix after marking route:")
        self.print_matrix()

if __name__ == "__main__":
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

    # Call the function to open the position input popup
    open_position_input_popup()
