import json
import math


def read_chs(data: str): # number + L * a + H * b
    return [float(data.split()[0]), float(data.split()[4]),
            float(data.split()[8])] # [number, a, b]


class Car:

    def __init__(self, kind):
        self.kind = kind
        self.way = []
        self.status = -1 # -1 - didn't start; 0 - on its way; 1 - reached destenation


class Road:
    
    def __init__(self, start, end, characteristics):
        self.start = start
        self.end = end
        self.characteristics = read_chs(characteristics)
        self.time = 0


def simulation():

    with open('cars.json') as jsonfile:
            cars = [Car(i) for i in json.load(jsonfile)] # list of L and H

    source = 'a' # graph['source']
    destination = 'c' # graph['destination']


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
            print(route)

            # new car start
            for car in cars:
                if car.status == -1:
                    car.way = route
                    car.status = 0
                    break

        # time decreasion for all driving cars (-1 iteration till all 0)
        for car in filter(lambda x: x.status == 0, cars):
            if all(map(lambda x: x[1] == 0, car.way)): # check if destination is reached
                cars.remove(car)
            else:
                for node in range(len(car.way)):
                    if car.way[node][1] - 1 >= 0:
                        car.way[node][1] -= 1
                        break
                    elif abs(car.way[node][1] - 1) < 1:
                        car.way[node][1] = 0
                        break
simulation()
