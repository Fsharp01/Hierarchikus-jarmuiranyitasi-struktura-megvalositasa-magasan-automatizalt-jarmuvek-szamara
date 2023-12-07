import numpy as np
from classes import MapUtils

'''
Script for the processing of logged traffic data for training
Handles traffic log one-by-one
Traffic output example structure:
simulation_cycle_number;road_id;lane_id;speed_vector;location_vector
'''

path = r'../import/Town05.xodr'
in_data = r'import/traffic_output_example.txt'
out_x = 'X_train_output'
out_y = 'Y_train_output'

# Number of NPCs generated for the traffic logging
num_of_npc = 300

map_reader = MapUtils.Utils(path)
carla_map, road_map = map_reader.load_map()

# Generation of road key
road_key = np.zeros((len(carla_map.roads)*2, 1))

road_index = 0

for road in carla_map.roads:
    road_key[road_index] = road.id
    road_index += 1
    road_key[road_index] = road.id
    road_index += 1

contains_data = True

# Parsing the traffic log file
with open(in_data, 'r') as f_open:
    lines = f_open.readlines()
    sum_col = int(np.floor(len(lines)/(num_of_npc + 1)))
    proc_data = np.zeros((len(carla_map.roads) * 2, sum_col))
    col_temp = -1
    for line in lines:
        line_elems = line.split(sep=';')
        contains_data = True
        if line_elems[0] == 'sim_cycle':
            col_temp += 1
            contains_data = False
            if col_temp == sum_col - 1:
                break
        if contains_data:
            speed_data = line_elems[3].split()
            speed_data[0] = float(speed_data[0][11:len(speed_data[0])-1])
            speed_data[1] = float(speed_data[1][2:len(speed_data[1])-1])
            speed_data[2] = float(speed_data[2][2:len(speed_data[2])-1])
            # Calculating and normalizing vehicle speed
            speed_temp = np.sqrt(np.sum(np.square(speed_data)))/34
            for index, key in enumerate(road_key):
                if key == int(line_elems[1]):
                    # Lane direction specific road score calculation
                    if int(line_elems[2]) > 0:
                        proc_data[index, col_temp] += (1/num_of_npc)*(1 - speed_temp)
                    else:
                        proc_data[index + 1, col_temp] += (1/num_of_npc)*(1 - speed_temp)

X_out = proc_data[:, 0:sum_col-2]
Y_out = proc_data[:, 1:sum_col-1]

np.save(out_x, X_out)
np.save(out_y, Y_out)
np.savetxt('road_key', road_key)
