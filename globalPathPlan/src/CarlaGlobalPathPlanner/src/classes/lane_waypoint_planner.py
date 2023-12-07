#!/usr/bin/env python3
"""
Last mod: Alejandro D. 3/5/2022
--> Refactoring using MapObject

Global waypoint planner.

Receives the ego vehicle position and the goal xyz in ROS Topics.
Calculates a new route as a list of waypoint centered at the lane every time
a new goal is published.

The corresponding marker to the route calculated is also published to visualize
the route in RVIZ.
"""
import sys
import os

import rospy
import networkx


from lane_graph_planner import LaneGraphPlanner

from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from map_object import MapObject


class LaneWaypointPlanner(LaneGraphPlanner):
    

    def __init__(self, map_data, map_path, distance=0.5, map_flag=0):
        """
        Args
            map_data: (str) It can be given in two ways:
                            0) Name of the map without extension, i.e. 'Town01' \
                            1) All map data saved as a string 
            map_path: (str) Path of the map files
            distance: (flt) Distance in meters between waypoints when calculating 
                            the route
            map_flag: (bool) 0) map_data in case 0
                             1) map data in case 1
        """

        LaneGraphPlanner.__init__(self, map_data, map_path, map_flag)
        """
        Self parameters
        """
        self.current_xyz = None
        self.current_waypoint = None
        self.current_waypoint = None
        self.goal_xyz = None
        self.distance = 2 
        self.previous_lane_id = None

        """
        ROS Subscribers
        """
          # ROS Subscribers for initial and goal positions
        self.startpos_subscriber = rospy.Subscriber(
            "/actual_pose", PoseStamped, self.startpos_callback)
        self.goal_subscriber = rospy.Subscriber(
            "/move_base_simple/goal", PoseStamped, self.goal_callback)
             
        """
        ROS Publishers
        """
        self.waypoint_publisher = rospy.Publisher(
            '/global_path', Path, queue_size=1, latch=True)


    def startpos_callback(self, startpos):
        """
        Callback for /initialpose topic
        """

        self.init_xyz = (
            startpos.pose.position.x,
            startpos.pose.position.y,
            startpos.pose.position.z
            #-59.978, -41.543, -1.495
        )
        #rospy.logwarn("startpos_callback")

    def goal_callback(self, goal):
        """
        Callback for /move_base_simple/goal topic
        """
        self.goal_xyz = (
            goal.pose.position.x,
            goal.pose.position.y,
            goal.pose.position.z
            #-81.820, -62.839, 0.000
        )
        rospy.logwarn("goal_callback")

        # Calculate route
        route = self.calculate_waypoint_route(
            self.distance, self.init_xyz, self.goal_xyz, weight="length")

        # Publish route
        self.publish_waypoints(route)
        rospy.logwarn(f"route size: {len(route)}")


    def publish_waypoints(self, route):
        """
        Publish a list of waypoints in /global_path
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = rospy.Time.now()

        
        if route is not None:
            for wp in route:
                waypoint_msg = PoseStamped()
                
                waypoint_msg.pose.position.x = wp.transform.location.x
                waypoint_msg.pose.position.y = wp.transform.location.y
                waypoint_msg.pose.position.z = 0

                #waypoint_msg.pose.orientation. = wp.transform.rotation.yaw
                path_msg.poses.append(waypoint_msg)

        self.waypoint_publisher.publish(path_msg)

        rospy.logwarn("Published {} waypoints.".format(len(path_msg.poses)))
        

    def calculate_waypoint_route(self, distance, init_xyz, goal_xyz, weight="length"):
        """
        Receives a route as a list of (road, lane, action) and generates 
        intermediate waypoints centered at the corresponding lane, given a 
        distance

        Args:
            distance: (int) Distance in meters to separate the waypoints
            init_xyz: (tuple) xyz values of the starting point of the route
            goal_xyz: (tuple) xyz values of the goal point of the route
            weight  : (str)   Weight to consider the cost in A* algorithm. 
                              Options are:
                                - length : distance in meters
                                - time   : length / vmax of the road

        Returns:
            waypoint_route: (list) Total route as a list of waypoints separated
                by a given distance
        """
        # lane_route: (list) Global route as a list of (road, lane, action)
        lane_route = self.calculate_global_route(init_xyz, goal_xyz, weight=weight)

        # roads: (list) road objects of the map where to get the waypoints from
        roads = self.map_object.roads

        # waypoints: (list) list of every waypoint cenetered at every lane
        # where to look for the init and the goal xyz
        waypoints = self.roadNetwork_waypoints

        waypoint_route = []

        # Convert xyz inputs to waypoints
        init_waypoint = self.map_object.get_waypoint(
            init_xyz[0], init_xyz[1], init_xyz[2])

        goal_waypoint = self.map_object.get_waypoint(
            goal_xyz[0], goal_xyz[1], goal_xyz[2])


        # This prevents when the route obtained is just one road/lane
        if len(lane_route) == 1:
            # Calculate waypoints until last waypoint of the route in the only step
            if lane_route[0][2] == "lanefollow":
                waypoint_route += self.calculate_waypoints_in_lane(
                    roads, lane_route[0][0], lane_route[0][1], 
                    distance, "last", goal_waypoint.s)

        elif len(lane_route) > 1:

            # Calculate waypoints from first waypoint of the route in first step 
            if lane_route[0][2] == "lanefollow":
                waypoint_route += self.calculate_waypoints_in_lane(
                        roads, lane_route[0][0], lane_route[0][1], 
                        distance, "first", init_waypoint.s)

            # Calculate waypoints for every other step of the route
            for i in range (1, len(lane_route)-1):
                if lane_route[i][2] == "lanefollow":
                    waypoint_route += self.calculate_waypoints_in_lane(
                            roads, lane_route[i][0], lane_route[i][1], 
                            distance, "middle", -1)


            # Calculate waypoints until last waypoint of the route in last step
            if lane_route[-1][2] == "lanefollow":
                waypoint_route += self.calculate_waypoints_in_lane(
                        roads, lane_route[-1][0], lane_route[-1][1], 
                        distance, "last", goal_waypoint.s)

        waypoint_route_filtered = []
        waypoint_route_filtered.append(waypoint_route[0])
        for waypoint in waypoint_route:
            distance_aux = self.euclidean_distance(
                    waypoint_route_filtered[-1], waypoint)
            # distance_aux = waypoint_route_filtered[-1].distance(waypoint)
            if distance_aux > (distance - 1):
                waypoint_route_filtered.append(waypoint)

        # Total cost of the route (just as relevant information)
        route_cost = 0
        for road_lane in lane_route:
            route_cost += self.road_weights[road_lane[0]][weight]

        return waypoint_route_filtered

    def calculate_waypoints_in_lane(self, roads, road_id, lane_id, 
                                    distance, position, s):
        """
        Generates waypoints centered in the lane using the corresponding
        functions

        Args:
            roads: (lst) Roads of the map object
            road_id: (int)
            lane_id: (int)
            distance: (flt) Distance between waypoints
            s: (flt) Only for first and last lanes
            position: (str) Position of the lane in the route. Options
                            are: first, middle or last

        Returns:
            waypoints: (list) A list of the calculated waypoints
        """
        road = self.map_object.get_road(roads, road_id)
        for j in range (len(road.lanes.laneSections)):
            lane = self.map_object.get_lane(road, lane_id, j)
            if lane and lane.type == "driving":
                break

        if position == "first":
            waypoints = self.map_object.generate_next_waypoints_until_lane_end(
                road, lane, distance, s)

        elif position == "middle":
            waypoints = self.map_object.generate_waypoints_in_lane(
                road, lane, distance)
        elif position == "last":
            waypoints = self.map_object.generate_previous_waypoints_until_lane_start(
                    road, lane, distance, s)
        else:
            raise Exception("Exception! Sorry, lane position must be first, " \
                            "middle or last") 
        
        if waypoints:
            return waypoints
        else:
            raise Exception("Waypoints could not be calculated for Road" \
                            " {}, lane {} ".format(road_id, lane_id))



    def calculate_waypoint_route_multiple(self, distance, input_data, mode, weight="length"):
        """
        This method has been developed for Carla Challenge 2021, where the input
        is a list of points instead of just one origin and one goal. 

        Receives a list of points, and returns a route of waypoints calculated from
        each to the next.

        It filters if there are more than one with same road and lane, to avoid errors 
        in planner.

        Args:
            distance  : (int) Distance in meters to separate the waypoints
            input_data: (list) It can be in two different formats depending on mode
            mode      : Mode can be 0 or 1 depending on the format of the input_data
                            0) Input is a list of tuples (x,y)
                            1) CarlaChallenge2021 format: A list of tuples 
                                                        (transform, transform)
            weight    : (str)   Weight to consider the cost in A* algorithm. 
                                Options are:
                                    - length : distance in meters
                                    . time   : length / vmax of the road

        Returns:
            waypoint_route_filtered
        """ 
        # roads: (list) road objects of the map where to get the waypoints from
        roads = self.map_object.roads
        # waypoints: (list) list of every waypoint cenetered at every lane
        # where to look for the init and the goal xyz
        waypoints = self.roadNetwork_waypoints

        # If mode == 1 apply conversion from CarlaChalleng2021
        # to standard input format
        if mode == 0:
            input_xy_tuples = input_data[:]
        elif mode == 1:
            input_xy_tuples = (
                [(tuple[0].location.x, -tuple[0].location.y, 
                tuple[0].location.z) for tuple in input_data])
            ### Saves the route for debugging ###
            # planning_root = "/workspace/team_code/catkin_ws/src/t4ac_planning_layer/"
            # routes_path = "t4ac_global_planner_ros/src/test_challenge/" 
            # file_name = "raw_routes.xml"
            # raw_routes_path = planning_root + routes_path + file_name
            # with open (raw_routes_path, "a") as f:
            #     f.write("""<route id="" town="Town0">\n""")
            #     for tuple in input_data:
            #         f.write("""   <waypoint x="{x}" y="{y}" z="{z}" />\n""".format(
            #             x = tuple[0].location.x, y = -tuple[0].location.y, z = 0))
            #     f.write("</route>\n")
            ### Saves the route for debugging ###
        
        ## --- Only for debugging --- ##
        input_waypoint_debug = [] 
        input_waypoint_debug_filtered = [] 
        for i in range(0, len(input_xy_tuples)):
            waypoint = self.map_object.get_waypoint(input_xy_tuples[i][0],
                input_xy_tuples[i][1], input_xy_tuples[i][2])
            input_waypoint_debug.append((waypoint.transform.location.x,       # Only for debugging
                waypoint.transform.location.y, waypoint.transform.location.z, # Only for debugging
                waypoint.road_id, waypoint.lane_id))                          # Only for debugging
            if waypoint.junction == -1:
                input_waypoint_debug_filtered.append((waypoint.transform.location.x,    # Only for debugging
                    waypoint.transform.location.y, waypoint.transform.location.z,       # Only for debugging
                    waypoint.road_id, waypoint.lane_id))                                # Only for debugging
                # print(input_waypoint_debug_filtered[-1])
        ## --- Only for debugging --- ##    
        
        
        input_waypoints = []
        
        input_waypoints.append(input_xy_tuples[0])
        for i in range(1, len(input_xy_tuples)-1):
            waypoint = self.map_object.get_waypoint(input_xy_tuples[i][0],
                input_xy_tuples[i][1], input_xy_tuples[i][2])
            if waypoint.junction == -1:
            # if waypoint.junction == -1 or waypoint.junction == 1932:
                input_waypoints.append((waypoint.transform.location.x,
                                        waypoint.transform.location.y,
                                        waypoint.transform.location.z))
        input_waypoints.append(input_xy_tuples[-1])


                

        init_xyz = input_waypoints[0]
        goal_xyz = input_waypoints[-1]

        lane_route_aux = []
        lane_route_aux2 = []
        waypoint_route = []

        

        # First obtain the complete route at road/lane level
        j = 0 # This is used to solve the exception of NoPath, ommiting
                  # calculating 1 to 2 and calculating next step from 1 to 3 
                  # instead of 2 to 3
        for i in range(0, len(input_waypoints)-1):
            try:
                lane_route_aux2 += self.calculate_global_route(
                    input_waypoints[i-j], input_waypoints[i+1], weight=weight)
                j = 0
            except (networkx.exception.NetworkXNoPath):
                j = 1
                print("Ooops!", sys.exc_info()[0], "occurred")

        # Then filter in case of repeated road/lane steps
        lane_route_aux.append(lane_route_aux2[0])
        for i in range(1, len(lane_route_aux2)):
            if lane_route_aux2[i] != lane_route_aux[-1]:
                lane_route_aux.append(lane_route_aux2[i])
        
        # For this CarlaChallenge2021, adapting the algorithm to the input 
        # waypoints data, delete also repeated roads for laneChange
        lane_route_aux = ([road_lane for road_lane in lane_route_aux if (
                                road_lane[2] == 'lanefollow')])
        lane_route = []
        lane_route.append(lane_route_aux[0])
        for i in range(0, len(lane_route_aux)-1):
            # Compare if road id is already in lane_route
            if (lane_route[-1][0] != lane_route_aux[i][0]):
                lane_route.append(lane_route_aux[i])
        lane_route.append(lane_route_aux[-1])


        # Convert xyz inputs to waypoints
        init_waypoint = self.map_object.get_waypoint(
            init_xyz[0], init_xyz[1], init_xyz[2])

        goal_waypoint = self.map_object.get_waypoint(
            goal_xyz[0], goal_xyz[1], goal_xyz[2])

        # Calculate waypoints from first waypoint of the route in first step 
        if lane_route[0][2] == "lanefollow":
            road = self.map_object.get_road(roads, lane_route[0][0])
            lane = self.map_object.get_lane(road, lane_route[0][1], 'unknown')
            waypoint_route += self.map_object.generate_next_waypoints_until_lane_end(
                road, lane, distance, init_waypoint.s)

        # Calculate waypoints for every other step of the route
        for i in range (1, len(lane_route)-1):
            if lane_route[i][2] == "lanefollow":
                road = self.map_object.get_road(roads, lane_route[i][0])
                lane = self.map_object.get_lane(road, lane_route[i][1], 'unknown')
                waypoint_route += self.map_object.generate_waypoints_in_lane(
                    road, lane, distance)

        # Calculate waypoints until last waypoint of the route in last step
        if lane_route[-1][2] == "lanefollow":
            road = self.map_object.get_road(roads, lane_route[-1][0])
            lane = self.map_object.get_lane(road, lane_route[-1][1], 'unknown')
            waypoint_route += self.map_object.generate_previous_waypoints_until_lane_start(
                road, lane, distance, goal_waypoint.s)

        # First filter too close waypoints
        waypoint_route_filtered_1 = []
        waypoint_route_filtered_1.append(waypoint_route[0])
        for waypoint in waypoint_route:
            distance_aux = self.euclidean_distance(
                    waypoint_route_filtered_1[-1], waypoint)
            # distance_aux = waypoint_route_filtered_1[-1].distance(waypoint)
            if distance_aux > (distance - 0.2):
                waypoint_route_filtered_1.append(waypoint)
        
        # Then filter paralel waypoints for lane changes
        waypoint_route_filtered_2 = []
        waypoint_route_filtered_2.append(waypoint_route_filtered_1[0])
        lane_change_flag = 0
        for waypoint in waypoint_route_filtered_1:
            distance_aux = self.euclidean_distance(
                    waypoint_route_filtered_2[-1], waypoint)
            if (distance_aux < 3 and distance_aux > (distance - 0.2)) or lane_change_flag == 2: # Distance_aux < 3 is a standard value for lane_width supposed to be > 3m
                lane_change_flag = 0
                waypoint_route_filtered_2.append(waypoint)
            else:
                lane_change_flag += 1


        # Total cost of the route (just as relevant information)
        route_cost = 0
        for road_lane in lane_route:
            route_cost += self.road_weights[road_lane[0]][weight]

        return waypoint_route_filtered_2


def main():
    """
    Main function
    """

    try:
        rospy.init_node("global_planner_node", anonymous=True)
        rospy.logwarn("Route planner initialized..")
        map_name = rospy.get_param('/t4ac/mapping/map_name')
        map_path = rospy.get_param('/t4ac/mapping/map_path')
        distance = float(rospy.get_param('/t4ac/mapping/distance_between_waypoints'))
        laneWaypointPlanner = LaneWaypointPlanner(map_name, map_path, distance)
        rospy.spin()

    except rospy.ROSException:
        rospy.logerr("Error while waiting for info!")
        sys.exit(1)

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
    
