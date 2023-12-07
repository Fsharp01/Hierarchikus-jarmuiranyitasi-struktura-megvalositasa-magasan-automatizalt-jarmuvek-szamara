"""
Written by Abel Kun
Classes that parse an .xodr file quickly as a python class
"""
from lxml import etree

from opendriveparser import parse_opendrive


class Road:
    def __init__(self, road_id, weight):
        self.road_id = road_id
        self.weight = weight
        self.total_length = float(0)
        self.traffic = int(0)
        self.segment_data = None

    def sum_segment(self, road_segment):
        self.total_length += road_segment.length*self.weight
        self.segment_data = road_segment

    def add_traffic(self, traffic_data):
        self.traffic += (traffic_data/int(self.total_length))


class RoadMap:
    def __init__(self):
        self.roads = dict()

    def add_road(self, index, road_id, weight):
        road_tmp = Road(road_id, weight)
        self.roads[index] = road_tmp


class Utils:
    def __init__(self, map_path):
        self.carla_map = None
        self.road_map = RoadMap()
        self.map_path = map_path
        self.default_weight = 1

    def load_map(self):

        with open(self.map_path, 'r') as fh:
            parser = etree.XMLParser()
            root_node = etree.parse(fh, parser).getroot()

            self.carla_map = parse_opendrive(root_node)

            self.fill_road_map()

        return self.carla_map, self.road_map

    def fill_road_map(self):

        for i, road in enumerate(self.carla_map.roads):
            id_tmp = road.id
            weight_tmp = self.default_weight
            self.road_map.add_road(i, id_tmp, weight_tmp)
