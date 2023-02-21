import json
import math
from tkinter import *


source = 'a' # graph['source']
destination = 'c' # graph['destination']

window = Tk()
canvas = Canvas(window, height=500, width=500)
canvas.pack()
canvas.configure(bg='white')


def read_chs(data: str): # number + L * a + H * b
    return [float(data.split()[0]), float(data.split()[4]),
            float(data.split()[8])] # [number, a, b]


class Car:

    def __init__(self, kind, canvas):
        self.canvas = canvas
        self.kind = kind
        self.way = [].copy()
        self.default_way = [].copy()
        self.status = -1 # -1 - didn't start; 0 - on its way; 1 - reached destenation
        if kind == 'L':
            self.image = self.canvas.create_oval(0, 0, 10, 10, fill='blue')
        else:
            self.image = self.canvas.create_rectangle(0, 0, 10, 10, fill='orange')

    def moveto(self):
        for i in range(len(self.way)):
            if self.way[i][1] > 0:
                self.canvas.moveto(self.image, (coords[self.way[i][0]][0] + coords[self.way[i - 1][0]][0]) *
                                (self.default_way[i][1] - self.way[i][1]) / self.default_way[i][1],
                                (coords[self.way[i][0]][1] + coords[self.way[i - 1][0]][1]) *
                                (self.default_way[i][1] - self.way[i][1]) / self.default_way[i][1])
                break


def graph_calculation(graph: dict):
    traffic = list(filter(lambda x: x.status == 0, cars))
    graph = graph.copy()
    for node in graph.keys():
        for car in traffic:
            for car_node in range(len(car.way) - 1):
                if node in car.way[car_node] and car.way[car_node + 1][0] in graph[node].keys() and car.way[car_node + 1][1] > 0:
                    if car.kind == 'L':
                        graph[node][car.way[car_node + 1][0]][1] += 1
                        break
                    else:
                        graph[node][car.way[car_node + 1][0]][2] += 1
                        break
    for node in graph.keys():
        if graph[node]:
            for next_node in graph[node].keys():
                graph[node][next_node][0] = read_chs(graph[node][next_node][0])[0] \
                    + graph[node][next_node][1] * read_chs(graph[node][next_node][0])[1] \
                    + graph[node][next_node][2] * read_chs(graph[node][next_node][0])[2]
    return graph


with open('roads.json') as jsonfile:
    graph = json.load(jsonfile) # {a: {b: [10 + L * 10 + H * 5, 0, 0] (current L and H cars number)}}
with open('cars.json') as jsonfile:
    cars = [Car(i, canvas) for i in json.load(jsonfile)] # list of L and H
with open('Nodes_coords.json') as jsonfile:
    coords = json.load(jsonfile)

# drawing map
for i in coords.keys():
    canvas.create_oval(coords[i][0] - 10, coords[i][1] - 10, coords[i][0] + 10, coords[i][1] + 10,
                        width=2, outline='black', fill='white')
    canvas.create_text(coords[i][0], coords[i][1],
                        text = i, font=("Arial", 15), fill='black')

while cars:
    with open('roads.json') as jsonfile:
            graph = json.load(jsonfile) # {a: {b: [10 + L * 10 + H * 5, 0, 0] (current L and H cars number)}}
    if any(map(lambda x: x.status == -1, cars)):
        # preparing data
        unvisited = graph_calculation(graph)
        shortest_distances = {}
        route = []
        path_nodes = {}

        # generating default setup and calculating edges
        for nodes in unvisited:
            shortest_distances[nodes] = math.inf
        shortest_distances[source] = 0.0

        # Dijkstra’s algorithm
        while unvisited:
            min_node = None
            for current_node in unvisited:
                if min_node is None:
                    min_node = current_node
                elif shortest_distances[min_node] > shortest_distances[current_node]:
                    min_node = current_node
            for (node, value) in unvisited[min_node].items():
                if value[0] + shortest_distances[min_node] < shortest_distances[node]:
                    shortest_distances[node] = value[0] + shortest_distances[min_node]
                    path_nodes[node] = [min_node, shortest_distances[min_node]]
            unvisited.pop(min_node)
        node, node_time = destination, shortest_distances[min_node]
            
        # storing data
        while node != source:
            route.insert(0, [node, node_time])
            node, node_time = path_nodes[node]
        for i in range(len(route) - 1, 0, -1):
            route[i][1] -= route[i - 1][1]
        route.insert(0, [source, 0])

        # new car start
        for car in cars:
            if car.status == -1:
                car.way = route.copy()
                car.default_way = route.copy()
                car.status = 0
                break
    
    # draw cars
    for car in filter(lambda x: x.status == 0,cars):
        car.moveto()
        

    # time decreasion for all driving cars (-1 iteration till all 0)
    for car in filter(lambda x: x.status == 0, cars):
        if all(map(lambda x: x[1] == 0, car.way)): # check if destination is reached
            cars.remove(car)
        else:
            for node in range(len(car.way)):
                if car.way[node][1] - 1 >= 0:
                    car.way[node][1] -= 1
                    print(car.default_way, car.way)
                    break
                elif abs(car.way[node][1] - 1) < 1:
                    car.way[node][1] = 0
                    print(car.default_way, car.way)
                    break
    window.update()
window.mainloop()
