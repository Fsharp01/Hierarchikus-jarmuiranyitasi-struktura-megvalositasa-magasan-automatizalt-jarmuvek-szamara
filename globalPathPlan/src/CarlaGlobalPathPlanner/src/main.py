from classes import MapUtils
from classes.lane_graph_planner import LaneGraphPlanner
from classes.lane_waypoint_planner import LaneWaypointPlanner
import numpy as np

# Path to map file (.xodr format)
map_path = r'import/Town05.xodr'
path = r'import/'

# Path to pre-trained model used for traffic prediction
# model_path = r'trained_models/town05_model'

# Key file containing road IDs
key_path = r'import/road_key'

# Traffic data for inference input
test_data = np.load(r'import/X_test.npy').T

map_reader = MapUtils.Utils(map_path)
carla_map, road_map = map_reader.load_map()

# Lane graph planner init
lane_graph_planner = LaneGraphPlanner('Town05', path)
waypoint_planner=LaneWaypointPlanner('Town05',path,0.5)

# FYI: Only the traffic weight uses the output of the traffic prediction
# Route number 1 - weight can be either length, time of traffic
#route = lane_graph_planner.calculate_global_route([202, -13, 0], [-125, -136, 0], weight='length')
waypoints=waypoint_planner.calculate_waypoint_route(0.5,[202, -13, 0], [-125, -136, 0], weight='length')
# Route number 2
#route = lane_graph_planner.calculate_waypoint_route(25, [-189, 105, 0], [202, -13, 0], weight='traffic')
# Route number 3
# route = lane_graph_planner.calculate_waypoint_route(25, [202, -13, 0], [114, 0, 0], weight='traffic')
out_path = r'_weight.txt'

f = open(out_path, 'w')

# Saving the waypoint route as txt
lane_graph_planner.save_route_to_txt(route, f)