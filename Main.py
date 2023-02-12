import json
import math


def read_chs(data: str): #number + L / a + H / b
    return [float(data.split()[0]), float(data.split()[4]),
            float(data.split()[8])] # [number, a, b]


class Car:

    def __init__(self, kind='L'):
        self.kind = kind
        self.way = []
        self.status = -1


class Road:
    
    def __init__(self, start, end, characteristics):
        self.start = start
        self.end = end
        self.characteristics = read_chs(characteristics)
        self.time = 0


def simulation():

    with open('roads.csv') as jsonfile:
        graph = json.load(jsonfile) # {a: {b: [10 + L / 10 + H / 5, 0, 0(current L and H cars number)]}}
    with open('cars.csv') as jsonfile:
        cars = [Car(i) for i in json.load(jsonfile)] # list of L and H

    def graph_calculation(graph: dict):
        traffic = filter(lambda x: x.status == -1, cars)
        for node in graph.keys():
            if not traffic:
                break
            else:
                for car in traffic:
                    for car_node in range(len(car) - 1):
                        if node in car[car_node] and car[car_node + 1][0] in graph[node].keys():
                            if car.kind == 'L':
                                graph[node][car_node + 1][1] += 1
                            else:
                                graph[node][car_node + 1][2] += 1
                            car.status = 0
                            traffic.pop(car)
        for node in graph.keys():
            for next_node in graph[node].keys():
                graph[node][next_node][0] = read_chs(graph[node][next_node][0])[0] \
                    + graph[node][next_node][1] / read_chs(graph[node][next_node][0])[1] \
                    + graph[node][next_node][2] / read_chs(graph[node][next_node][0])[2]
        return graph


    source = graph['source']
    destination = graph['destination']

    while not all(map(lambda x: x.status == 1, cars)):
        
        # preparing data
        unvisited = graph_calculation(graph)
        shortest_distances = {}
        route = []
        path_nodes = {}

        # generating default setup and calculating edges
        for nodes in unvisited:
            shortest_distances[nodes] = math.inf
        shortest_distances[source] = 0

        # Dijkstra’s algorithm
        while unvisited:
            min_node = None
            for current_node in unvisited:
                if min_node is None:
                    min_node = current_node
                elif shortest_distances[min_node] > shortest_distances[current_node]:
                    min_node = current_node
            for (node, value) in unvisited[min_node].items():
                if value + shortest_distances[min_node] < shortest_distances[node]:
                    shortest_distances[node] = value + shortest_distances[min_node]
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
                car.way = route
                car.status = 0
                break
