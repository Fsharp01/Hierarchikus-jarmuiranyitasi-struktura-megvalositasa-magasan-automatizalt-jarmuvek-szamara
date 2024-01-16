#!/usr/bin/env python3

"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import tf
import pathlib
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
sys.path.append(str(pathlib.Path(__file__).parent.parent))


from quintic_polynomials_planner import QuinticPolynomial
from cubic_spline_planner import CubicSpline1D
from cubic_spline_planner import CubicSpline2D
import cubic_spline_planner

SIM_LOOP = 500
#Stanley Parameter
k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time difference
L = 2.79  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle


# Parameter
MAX_SPEED = 50.0 / 3.6  # maximum speed [m/s]
MAX_ACCEL = 2.0  # maximum acceleration [m/ss]
MAX_CURVATURE = 10.0  # maximum curvature [1/m]
MAX_ROAD_WIDTH = 1.0  # maximum road width [m]
D_ROAD_W = 1.0  # road width sampling length [m]
DT = 1  # time tick [s]
MAX_T = 5.0  # max prediction time [m]
MIN_T = 2.0  # min prediction time [m]
TARGET_SPEED = 30.0 / 3.6  # target speed [m/s]
D_T_S = 5.0 / 3.6  # target speed sampling length [m/s]
N_S_SAMPLE = 1  # sampling number of target speed
ROBOT_RADIUS = 3.0  # robot radius [m]

# cost weights
K_J = 0.1
K_T = 0.1
K_D = 1.0
K_LAT = 1.0
K_LON = 1.0

show_animation = True


class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt

class State(object):
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        super(State, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.ax=None
        self.ay=None
        self.control_enabled=False
        self.waypoints=None

    def update(self, acceleration, delta):
        """
        Update the state of the vehicle.

        Stanley Control uses bicycle model.

        :param acceleration: (float) Acceleration
        :param delta: (float) Steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt

    def getLocalpath(self,msg):
        self.waypoints = msg
        waypoints = self.extract_waypoints()
        self.ax, self.ay, wz =zip(*waypoints)
        rospy.logwarn(len(self.ax))
        self.control_enabled=True
        
    def extract_waypoints(self):
            if self.waypoints is None:
                rospy.logwarn("No global path received. Cannot extract waypoints.")
                return None

            waypoints = []
            for pose_stamped in self.waypoints.poses:
                waypoint = pose_stamped.pose
                x = waypoint.position.x
                y = waypoint.position.y
                z = waypoint.position.z
                rospy.logwarn('pozicio érkezik')
                waypoints.append((x, y, z))  # Adatok hozzáadása a waypoints listához
            rospy.logwarn("Subscribed {} waypoints.".format(len(waypoints)))
            return waypoints

def pid_control(target, current):
    """
    Proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return Kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + L * np.cos(state.yaw)
    fy = state.y + L * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle    

class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []
class FrenetOptimalPlannerNode:
    def __init__(self):
        rospy.init_node('frenet_optimal_planner_node', anonymous=True)
        self.global_path_sub = rospy.Subscriber('/global_path', Path, self.global_path_callback)
        #self.global_path_sub = rospy.Subscriber('/actual_pose', PoseStamped, self.get_actualPose_callback)

        self.local_path_pub = rospy.Publisher('/local_path', Path, queue_size=1)
        self.global_path = None
        self.frenet_planning_enabled = False
        self.csp = None  # Initialize csp here
        self.tc= None
        self.tx= None
        self.ty= None
        self.tyaw= None
        self.robotpos_x=None
        self.robotpos_y=None
        self.robot_theta=None
    
    def find_closest_point(self, path,vehicle_state):
        min_distance = float('inf')
        closest_index = None

        for i in range(len(path.x)):
            distance = np.sqrt(np.power((path.x[i] - vehicle_state.x),2) + np.power((path.y[i] - vehicle_state.y),2))
            if distance < min_distance:
                min_distance = distance
                closest_index = i

        return closest_index, min_distance

    def get_actualPose_callback(self,msg):
        # Access the pose data from the message
        self.robotpos_x = msg.pose.position.x
        self.robotpos_y = msg.pose.position.y
        quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        eulZYX = tf.transformations.euler_from_quaternion(quat)
        self.robot_theta = eulZYX[2]
    def global_path_callback(self, msg):
        self.global_path = msg
        waypoints = self.extract_waypoints()
        wx, wy, wz = zip(*waypoints)
        self.tx, self.ty, self.tyaw, self.tc, self.csp = generate_target_course(wx, wy)
        self.frenet_planning_enabled = True
    def extract_waypoints(self):
        if self.global_path is None:
            rospy.logwarn("No global path received. Cannot extract waypoints.")
            return None

        waypoints = []
        for pose_stamped in self.global_path.poses:
            waypoint = pose_stamped.pose
            x = waypoint.position.x
            y = waypoint.position.y
            z = waypoint.position.z
            waypoints.append((x, y, z))  # Adatok hozzáadása a waypoints listához
        rospy.logwarn("Subscribed {} waypoints.".format(len(waypoints)))
        return waypoints
    
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
                
                waypoint_msg.pose.position.x = wp.x
                waypoint_msg.pose.position.y = wp.y
                waypoint_msg.pose.position.z = 0

                #waypoint_msg.pose.orientation. = wp.transform.rotation.yaw
                path_msg.poses.append(waypoint_msg)

        self.local_path_pub.publish(path_msg)

        
    

    
    def run(self):
        br = tf.TransformBroadcaster()
        rate = rospy.Rate(100)  # 100 Hz
         # initial state
        target_speed = 30.0 / 3.6  # [m/s]
        c_speed = target_speed  # current speed [m/s]
        c_accel = 0.0  # current acceleration [m/ss]
        c_d = 0.0  # current lateral position [m]
        c_d_d = 0.0  # current lateral speed [m/s]
        c_d_dd = 0.0  # current lateral acceleration [m/s]
        s0 = 0.0  # current course position
        area = 2000.0  # animation area length [m]
        ob = np.array([[100.0, 10.0],
                    [130.0, 6.0],
                    [200.0, 8.0],
                    [355.0, 8.0]
                    ])
        firstplan=False
        
        state = State(x=-3.421, y=-8.410, yaw=np.radians(100.0), v=0.0)
        while(self.frenet_planning_enabled==False):
            pass
        path = frenet_optimal_planning(self.csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob)
        while not rospy.is_shutdown():
            #rospy.logwarn("Frenet plan start")
          
            

            target_idx, _ = calc_target_index(state, path.x, path.y)
            ai = pid_control(target_speed, state.v)
            di, target_idx = stanley_control(state, path.x, path.y, path.yaw, target_idx)
            state.update(ai, di)

            
            completion_percentage = closest_index / len(path.x) * 100.0
            rospy.logwarn(min_distance, closest_index, completion_percentage)
            if completion_percentage >= 90.0:
                closest_index, min_distance=self.find_closest_point(path, state)
                s0 = path.s[closest_index]   
                c_d = min_distance
                path = frenet_optimal_planning(self.csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob)         
            
            rospy.logwarn("local path")
            rospy.logwarn(len(path.x))
            # Convert best_path to ROS Path message
            local_path_msg_ros = Path()
            local_path_msg_ros.header.frame_id = "map"
            if len(path.x):
                #self.publish_waypoints(path)
                for i in range(len(path.x)):
                                        pose = PoseStamped()
                                        pose.pose.position.x = path.x[i]
                                        pose.pose.position.y = path.y[i]
                                        local_path_msg_ros.poses.append(pose)

                # Publish local path
                self.local_path_pub.publish(local_path_msg_ros)
                br.sendTransform((state.x, state.y, 0),
                                tf.transformations.quaternion_from_euler(0, 0,state.yaw),
                                rospy.Time.now(),
                                "base_link",
                                "map")     
            


                    
                    

            rate.sleep()    

def calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in np.arange(MIN_T, MAX_T, DT):
            fp = FrenetPath()

            # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = [t for t in np.arange(0.0, Ti, DT)]
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE,
                                TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S):
                tfp = copy.deepcopy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, c_accel, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = (TARGET_SPEED - tfp.s_d[-1]) ** 2

                tfp.cd = K_J * Jp + K_T * Ti + K_D * tfp.d[-1] ** 2
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths


def calc_global_paths(fplist, csp):

    for fp in fplist:

        # calc global positions
        for i in range(len(fp.s)):
            ix, iy = csp.calc_position(fp.s[i])
            if ix is None:
                break
            i_yaw = csp.calc_yaw(fp.s[i])
            di = fp.d[i]
            fx = ix + di * math.cos(i_yaw + math.pi / 2.0)
            fy = iy + di * math.sin(i_yaw + math.pi / 2.0)
            fp.x.append(fx)
            fp.y.append(fy)

        # Check if fp.yaw is not empty before appending the last element
        if fp.yaw:
            fp.yaw.append(fp.yaw[-1])
        else:
            # Handle the case when fp.yaw is empty (e.g., no points generated)
            fp.yaw.append(0.0)  # You may adjust this default value

        # Check if fp.ds is not empty before appending the last element
        if fp.ds:
            fp.ds.append(fp.ds[-1])
        else:
            # Handle the case when fp.ds is empty (e.g., no points generated)
            fp.ds.append(0.0)  # You may adjust this default value

        # calc curvature
        for i in range(len(fp.yaw) - 1):
            fp.c.append((fp.yaw[i + 1] - fp.yaw[i]) / fp.ds[i])

    return fplist


def check_collision(fp, ob):
    for i in range(len(ob[:, 0])):
        d = [((ix - ob[i, 0]) ** 2 + (iy - ob[i, 1]) ** 2)
             for (ix, iy) in zip(fp.x, fp.y)]

        collision = any([di <= ROBOT_RADIUS ** 2 for di in d])

        if collision:
            return False

    return True


def check_paths(fplist, ob):
    ok_ind = []
    for i, _ in enumerate(fplist):
        if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
            continue
        elif any([abs(a) > MAX_ACCEL for a in
                  fplist[i].s_dd]):  # Max accel check
            continue
        elif any([abs(c) > MAX_CURVATURE for c in
                  fplist[i].c]):  # Max curvature check
            continue
        elif not check_collision(fplist[i], ob):
            continue

        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]


def frenet_optimal_planning(csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob):
    rospy.logwarn("Frenet Optimal Planning")
    fplist = calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0)
    rospy.logwarn("Calc frenet paths")
    fplist = calc_global_paths(fplist, csp)
    rospy.logwarn("Calc global paths")

    fplist = check_paths(fplist, ob)

    # find minimum cost path
    min_cost = float("inf")
    best_path = None
    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp

    return best_path


def generate_target_course(x, y):
    
    csp = cubic_spline_planner.CubicSpline2D(x, y)
    if csp is None:
        print("Error: Failed to generate CubicSpline2D.")
        return None

    s = np.arange(0, csp.s[-1], 0.1)
    rx, ry, ryaw, rk = [], [], [], []
    
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))
    rospy.logwarn("Generate target course")

    return rx, ry, ryaw, rk, csp


 
if __name__ == '__main__':
    try:
        planner_node = FrenetOptimalPlannerNode()
        planner_node.run()
    except rospy.ROSInterruptException:
        pass