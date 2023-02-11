import csv


def read_chs(data: str): #number + L / a + H / b
    return [data.split()[0], data.split()[4], data.split()[8]] # [number, a, b]


class Car:

    def __init__(self, kind='Light'):
        self.kind = kind
        self.way = []


class Road:
    
    def __init__(self, start, end, characteristics):
        self.start = start
        self.end = end
        self.characteristics = read_chs(characteristics)
        self.time = 0


def simulation():

    with open('roads.csv') as csvfile:
        roads = [Road(*i) for i in csv.reader(csvfile, delimiter=',', quotechar='"')]
    with open('cars.csv') as csvfile:
        cars = [Car(*i) for i in csv.reader(csvfile, delimiter=',', quotechar='"')]
