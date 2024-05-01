#TO RUN THIS CODE YOU MUST USE TERMINAL COMMANDS OF 'python main2.py <start_row> <start_col> <end_row> <end_col>'

import heapq
import sys
import numpy as np
import cv2
import tkinter as tk
import time
import tkinter.messagebox as messagebox
import queue

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
        
        self.priority_queue = queue.PriorityQueue()

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

        # Display the matrix with routes marked in a popup
        root = tk.Tk()
        popup = MatrixDisplayPopup(root, delivery_system.matrix)

        # Calculate routes and mark them on the matrix for each destination
        routes_info = ""
        for destination in destination_positions:
            # Calculate routes using Dijkstra's algorithm and A* algorithm
            dijkstra_route = delivery_system.dijkstra(start_pos, destination)
            astar_route = delivery_system.astar(start_pos, destination)

            popup.display_path(dijkstra_route)
            popup.display_path(astar_route)

            # Choose the fastest route
            if dijkstra_route is None and astar_route is None:
                route_info = f"No valid routes found."
            elif dijkstra_route is None:
                route_info = f"Route using A* algorithm for destination {destination}: {astar_route}\n"
                delivery_system.mark_route_on_matrix(astar_route, 15)
            elif astar_route is None:
                route_info = f"Route using Dijkstra's algorithm for destination {destination}: {dijkstra_route}\n"
                delivery_system.mark_route_on_matrix(dijkstra_route, 16)
            else:
                if len(dijkstra_route) <= len(astar_route):
                    route_info = f"Route using Dijkstra's algorithm (shorter or equal) for destination {destination}: {dijkstra_route}\n"
                    delivery_system.mark_route_on_matrix(dijkstra_route, 16)
                else:
                    route_info = f"Route using A* algorithm (shorter) for destination {destination}: {astar_route}\n"
                    delivery_system.mark_route_on_matrix(astar_route, 15)
            routes_info += route_info

        messagebox.showinfo("Routes Information", routes_info)
        print("\nMatrix with routes marked:")
        delivery_system.print_matrix()
        self.master.destroy()

def calculate_priority(matrix, destination):
    dest_number = matrix[destination[0]][destination[1]]
    if dest_number == 1:
        return 5
    elif dest_number == 2:
        return 4
    elif dest_number == 3:
        return 3
    elif dest_number == 4:
        return 2
    else:
        return 1
    
class MatrixDisplayPopup:
    def __init__(self, master, matrix):
        self.master = master
        self.master.title("Matrix Display")
        
        self.matrix = matrix
        self.rows = len(matrix)
        self.cols = len(matrix[0])
        
        self.canvas = tk.Canvas(master, width=self.cols * 20, height=self.rows * 20)
        self.canvas.pack()
        
        colors = ['white', 'gray', 'red', 'yellow', 'blue', 'pink', 'dark green', 'orange', 'silver', 'green', '#FFCCCC', 'purple', 'light green', 'black', 'black', '#8B4513', '#00FFFF']


        for i in range(self.rows):
            for j in range(self.cols):
                color_index = min(self.matrix[i][j], len(colors) - 1)  # Ensure color index doesn't exceed the length of colors list
                color = colors[color_index]
                self.canvas.create_rectangle(j * 20, i * 20, (j + 1) * 20, (i + 1) * 20, fill=color)
        
    def display_path(self, path):
        if path is None:
            print("No path found.")
            return

        for i, (row, col) in enumerate(path):
            if i == 0 or i == len(path) - 1:
                color = "cyan"  # Start and destination nodes
            else:
                color = "brown"  # Path nodes

            self.canvas.create_rectangle(col * 20, row * 20, (col + 1) * 20, (row + 1) * 20, fill=color)
            self.master.update()
            time.sleep(0.5)

    def display_matrix(self):
        pass

    def open_matrix_display_popup(matrix):
        root = tk.Tk()
        popup = MatrixDisplayPopup(root, matrix)
        root.mainloop()
    
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
                if 0 <= x < self.rows and 0 <= y < self.cols and self.matrix[x][y] != 14:
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
    matrix = [    
[0, 0, 0, 14, 4, 4, 4, 4, 4, 4, 4, 4, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 14, 4, 4, 4, 4, 4, 4, 4, 4, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 14, 4, 4, 4, 4, 4, 4, 4, 4, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 14, 4, 4, 4, 4, 4, 4, 4, 4, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 14, 4, 4, 4, 4, 4, 4, 4, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 14, 4, 4, 4, 4, 4, 4, 4, 14, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 14, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 14, 4, 4, 4, 4, 4, 14, 14, 14, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
[0, 0, 0, 14, 4, 0, 0, 14, 14, 14, 14, 14, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14],
[14, 14, 14, 14, 14, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 14, 3, 3, 3, 3, 14, 1, 1, 14],
[14, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 14, 14, 0, 0, 0, 14, 3, 3, 3, 3, 14, 1, 1, 14],
[14, 0, 0, 0, 14, 8, 14, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 14, 8, 8, 14, 0, 0, 3, 3, 3, 3, 14, 1, 1, 14],
[14, 0, 0, 0, 14, 8, 8, 14, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 14, 8, 8, 14, 0, 0, 3, 3, 3, 3, 14, 1, 1, 14],
[14, 0, 0, 0, 14, 8, 8, 14, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 14, 8, 8, 14, 0, 14, 3, 3, 3, 3, 14, 1, 1, 14],
[14, 0, 0, 0, 0, 14, 0, 0, 14, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 14, 14, 14, 14, 14, 8, 8, 14, 0, 14, 3, 3, 3, 3, 14, 1, 1, 14],
[14, 0, 0, 0, 0, 0, 0, 0, 14, 2, 2, 2, 2, 2, 2, 2, 14, 14, 14, 2, 2, 2, 2, 14, 3, 3, 8, 8, 8, 14, 0, 14, 3, 3, 3, 3, 14, 1, 1, 14],
[14, 0, 0, 0, 0, 14, 6, 6, 14, 14, 2, 2, 2, 2, 2, 2, 14, 10, 14, 2, 2, 2, 2, 14, 3, 3, 8, 8, 8, 14, 0, 14, 14, 14, 1, 1, 1, 1, 1, 14],
[14, 0, 0, 0, 0, 14, 6, 6, 6, 14, 2, 2, 2, 2, 2, 2, 14, 10, 14, 14, 2, 2, 2, 14, 3, 14, 3, 3, 3, 14, 0, 0, 7, 7, 1, 1, 1, 1, 1, 14],
[14, 0, 0, 0, 0, 14, 6, 6, 6, 14, 2, 2, 2, 2, 2, 2, 14, 10, 10, 14, 2, 2, 2, 14, 14, 6, 3, 3, 3, 14, 0, 14, 7, 7, 14, 14, 14, 14, 14, 14],
[14, 0, 0, 0, 0, 14, 6, 6, 6, 14, 14, 14, 14, 14, 14, 14, 10, 10, 10, 14, 2, 2, 2, 2, 14, 6, 6, 3, 3, 14, 0, 14, 7, 7, 7, 7, 7, 7, 7, 14],
[14, 0, 0, 0, 0, 14, 6, 6, 6, 14, 10, 10, 10, 10, 10, 10, 10, 10, 10, 14, 2, 2, 2, 2, 8, 6, 6, 6, 6, 14, 0, 14, 7, 7, 7, 7, 7, 7, 7, 14],
[14, 0, 0, 0, 0, 14, 6, 6, 6, 14, 10, 10, 10, 10, 10, 10, 10, 10, 10, 14, 2, 2, 0, 0, 8, 6, 6, 6, 6, 14, 0, 14, 7, 7, 7, 7, 7, 7, 7, 14],
[14, 0, 0, 0, 0, 14, 14, 14, 14, 14, 14, 0, 14, 14, 14, 14, 0, 0, 0, 0, 14, 14, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 7, 7, 7, 7, 7, 14],
[14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 14, 14, 0, 14, 14, 0, 0, 14, 14, 14, 14, 14, 14, 14, 14],
[14, 14, 14, 0, 0, 0, 0, 8, 6, 6, 14, 1, 1, 14, 14, 14, 11, 11, 11, 11, 11, 11, 14, 0, 0, 5, 5, 5, 5, 14, 0, 0, 6, 6, 6, 6, 6, 14, 14, 14],
[0, 0, 14, 0, 0, 0, 0, 6, 6, 6, 14, 1, 1, 1, 1, 14, 11, 11, 11, 11, 11, 11, 14, 0, 14, 5, 5, 5, 5, 14, 6, 6, 6, 6, 6, 6, 6, 14, 0, 0],
[0, 0, 14, 0, 0, 0, 14, 6, 6, 6, 14, 14, 14, 14, 14, 14, 11, 11, 11, 11, 11, 11, 14, 0, 14, 5, 5, 5, 5, 14, 6, 6, 6, 6, 6, 6, 6, 14, 0, 0],
[0, 0, 14, 0, 0, 0, 14, 6, 6, 6, 6, 6, 14, 9, 9, 11, 11, 11, 11, 11, 14, 14, 14, 0, 0, 5, 5, 5, 5, 14, 6, 6, 6, 6, 6, 6, 6, 14, 0, 0],
[0, 0, 14, 0, 0, 0, 14, 6, 6, 6, 6, 6, 14, 9, 9, 14, 14, 14, 14, 14, 14, 9, 14, 0, 0, 5, 5, 5, 5, 14, 6, 6, 6, 6, 6, 6, 6, 14, 0, 0],
[0, 0, 14, 0, 0, 0, 14, 6, 6, 6, 14, 14, 14, 9, 9, 9, 9, 9, 9, 9, 9, 9, 14, 0, 14, 5, 5, 5, 5, 14, 6, 6, 6, 6, 6, 6, 6, 14, 0, 0],
[0, 0, 14, 0, 0, 0, 14, 6, 6, 6, 14, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 14, 0, 14, 5, 5, 5, 5, 14, 14, 14, 14, 14, 14, 14, 14, 14, 0, 0],
[0, 0, 14, 0, 0, 0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 0, 14, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 14, 0, 0],
[0, 0, 14, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 14, 0, 0],
[0, 0, 14, 0, 0, 14, 0, 14, 14, 14, 14, 0, 0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 0, 0, 5, 5, 5, 5, 5, 5, 5, 5, 14, 14, 14, 14, 14, 0, 0],
[0, 0, 14, 0, 0, 6, 6, 14, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 14, 5, 5, 14, 14, 5, 12, 14, 5, 5, 5, 5, 5, 14, 0, 0],
[0, 0, 14, 0, 0, 6, 6, 14, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 14, 5, 5, 5, 14, 12, 12, 14, 5, 5, 5, 5, 5, 14, 0, 0],
[0, 0, 14, 14, 0, 0, 0, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 14, 5, 5, 5, 14, 12, 12, 14, 5, 5, 5, 5, 5, 14, 0, 0],
[0, 0, 14, 8, 8, 8, 8, 14, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 14, 5, 5, 5, 14, 12, 12, 14, 5, 5, 5, 5, 5, 14, 0, 0],
[0, 0, 14, 8, 8, 8, 8, 14, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 14, 5, 5, 5, 14, 12, 12, 14, 5, 5, 5, 5, 5, 14, 0, 0],
[0, 0, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 0, 0],
    ]


    delivery_system = RobotDeliverySystem(matrix)

    # Call the function to open the position input popup
    open_position_input_popup()
