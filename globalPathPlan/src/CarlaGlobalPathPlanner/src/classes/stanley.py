#!/usr/bin/env python3

"""
Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import numpy as np
import rospy
import matplotlib.pyplot as plt
import sys
import pathlib
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
sys.path.append(str(pathlib.Path(__file__).parent.parent.parent))

from cubic_spline_planner import CubicSpline1D
from cubic_spline_planner import CubicSpline2D
import cubic_spline_planner

k = 0.5  # control gain
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time difference
L = 2.79  # [m] Wheel base of vehicle
max_steer = np.radians(30.0)  # [rad] max steering angle

show_animation = True


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


def main():
    """Plot an example of Stanley steering control on a cubic spline."""

    rospy.init_node('path_tracking_node')
    br = tf.TransformBroadcaster()

    state = State(x=-3.421, y=-8.410, yaw=np.radians(100.0), v=0.0)

    local_path_pub = rospy.Subscriber('/global_path', Path, state.getLocalpath)
    actual_pose_pub= rospy.Publisher('/actual_pose',PoseStamped,queue_size=10)

       

    target_speed = 30.0 / 3.6  # [m/s]

    # Initial state
    rate=rospy.Rate(100)
    if state.waypoints is not None:
        rospy.logwarn("stanley control")
        cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
            state.ax, state.ay, ds=0.1)
        last_idx = len(cx) - 1
        target_idx, _ = calc_target_index(state, cx, cy)
    time = 0.0
    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    pose=PoseStamped()
    pose.header.frame_id="map"
    pose.header.stamp=rospy.Time.now()
    pose.pose.position.x=state.x
    pose.pose.position.y=state.y
    pose.pose.orientation.z=state.yaw
            
    actual_pose_pub.publish(pose)

    while not rospy.is_shutdown():
        pose=PoseStamped()
        pose.header.frame_id="map"
        pose.header.stamp=rospy.Time.now()
        pose.pose.position.x=state.x
        pose.pose.position.y=state.y
        pose.pose.orientation.z=state.yaw
        actual_pose_pub.publish(pose)
        br.sendTransform((state.x, state.y, 0),
                                    tf.transformations.quaternion_from_euler(0, 0, state.yaw),
                                    rospy.Time.now(),
                                    "base_link",
                                    "map")   
        
        if state.ax is not None:
            rospy.logwarn("Stanley control")
            cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(state.ax, state.ay, ds=0.1)
            last_idx = len(cx) - 1
            target_idx, _ = calc_target_index(state, cx, cy)
            ai = pid_control(target_speed, state.v)
            di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
            state.update(ai, di)

            time += dt
            """
            if last_idx-2<target_idx:
                state.control_enabled=False
                cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
                state.ax, state.ay, ds=0.1)
                last_idx = len(cx) - 1
                target_idx, _ = calc_target_index(state, cx, cy)
                """
        rate.sleep()

    

if __name__ == '__main__':
    main()