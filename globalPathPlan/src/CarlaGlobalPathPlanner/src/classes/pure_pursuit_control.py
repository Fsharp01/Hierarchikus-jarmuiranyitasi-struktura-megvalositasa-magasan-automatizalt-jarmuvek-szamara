#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
import tf
import numpy as np

# Define global variables
startposx = None
startposy = None
path = None
d_last_target = 0.0
target_index = 0
path_ready_flag = False
pub = None
theta = 0.0
twistMsg = Twist()
xG = None
yG = None
last_point = None  # Initialize last_point
#listener = tf.TransformListener()
rospy.logwarn("init")


def find_target_index(path, x2, y2, lookahead_distance, theta):
    target_index = 0  # Initialize target index to the first point in the path

    for i in range(len(path)):
        # Calculate the distance from the robot to the current point in the path
        d = np.sqrt((path[i, 0] - x2)**2 + (path[i, 1] - y2)**2)

        # Check if the current point is ahead of the robot and within the lookahead distance
        if d > lookahead_distance and np.dot([path[i, 0] - x2, path[i, 1] - y2], [np.cos(theta), np.sin(theta)]) > 0:
            # Update the target index to the current point in the path
            target_index = i
            break

    return target_index


def calculate_steering_angle(path, startposx, startposy, theta):
    global d_last_target, lookahead_distance, target_index, last_point

    x2 = startposx
    y2 = startposy
    steering_angle_list = []
    delta_list = []
    x_list = [x2]
    y_list = [y2]

    lookahead_distance = 4
    max_speed = 0.05
    dt = 0.1
    L = 2.790  # Wheelbase length
    max_steering_angle = np.pi / 4

    if last_point is None:
        last_point = path[-1, :]

    target_index = find_target_index(path, x2, y2, lookahead_distance, theta)

    x_target = path[target_index, 0]
    y_target = path[target_index, 1]

    d_target = np.linalg.norm([x_target - x2, y_target - y2])

    d_last_target = np.linalg.norm([last_point[0] - x2, last_point[1] - y2])

    alpha = np.arctan2(y_target - y2, x_target - x2)

    desired_steering_angle = np.arctan2(2 * L * np.sin(alpha - theta), d_target)

    steering_angle = np.clip(desired_steering_angle, -max_steering_angle, max_steering_angle)

    theta_dot = max_speed * np.tan(steering_angle) / L

    x_list.append(x2)
    y_list.append(y2)

    steering_angle_list.append(desired_steering_angle)
    delta_list.append(theta_dot)
    rospy.logwarn("steering angle")

    return steering_angle

def amcl_callback(msg):
    global startposx, startposy, path, d_last_target, lookahead_distance, target_index, path_ready_flag
    global pub, theta, twistMsg, xG, yG


    # Access the pose data from the message
    positionX = msg.pose.position.x
    positionY = msg.pose.position.y
    quat = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
    eulZYX = tf.transformations.euler_from_quaternion(quat)
    theta = eulZYX[2]

    # Update robot_pos matrix
    startposx = positionX
    startposy = positionY
"""
def odom_callback(odom_msg):
    global startposx, startposy, theta

    try:
        # Lookup the static transform from agent/odom to base_link
        (trans, rot) = listener.lookupTransform('/agent/odom', '/base_link', rospy.Time(0))

        # Extract odometry information
        delta_x = odom_msg.twist.twist.linear.x
        delta_y = odom_msg.twist.twist.linear.y
        delta_theta = odom_msg.twist.twist.angular.z

        # Update the robot's position and orientation
        startposx += delta_x
        startposy += delta_y
        theta += delta_theta

        # Transform the robot's position based on the static transform
        transformed_pos = listener.transformPoint('/base_link', (startposx, startposy, 0.0))
        startposx = transformed_pos[0]
        startposy = transformed_pos[1]

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Transform lookup failed")

    """
def local_path_callback(msg):
    global path, path_ready_flag, xG, yG

    # Update the path variable with the received local path
    path = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in msg.poses])
    
    # Set the path_ready_flag to True
    xG = path[target_index, 0]
    yG = path[target_index, 1]
    path_ready_flag = True    

def send_ros_message():
    global startposx, startposy, path, d_last_target, lookahead_distance, target_index
    global path_ready_flag, pub, theta, twistMsg, xG, yG

    goal_reached = False
    max_speed = 0.05

    # Start the timer
    if path_ready_flag:
        Gx = xG
        Gy = yG

        steering_angle = calculate_steering_angle(path, startposx, startposy, theta)

        # Uncomment the following section if the path_ready_flag is used
        # if d_last_target < lookahead_distance or target_index > len(path):
        #     goal_reached = True
        #     twistMsg.linear.x = 0
        #     twistMsg.angular.z = 0
        #     path_ready_flag = False
        twistMsg.linear.x = max_speed
        twistMsg.angular.z = steering_angle
        

        if np.linalg.norm([startposx - Gx, startposy - Gy]) <= 0.5:
            twistMsg.linear.x = 0
            twistMsg.angular.z = 0
            path_ready_flag = False

        # Publish the Twist message to the '/cmd_vel' topic
        pub.publish(twistMsg)   

def main():
    global pub

    # Initialize ROS node
    rospy.init_node('pure_pursuit_control_node')

    # Create a subscriber for the pose topic
    rospy.Subscriber('/actual_pose', PoseStamped, amcl_callback)

    # Create a publisher for the twist topic
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Create a subscriber for the local path topic
    rospy.Subscriber('/global_path', Path, local_path_callback)

    # rospy.Subscriber('/agent2/odom', Odometry, odom_callback)

    # Set the rate at which to run the main loop
    rate = rospy.Rate(10)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        if path_ready_flag==True:
        # Call the function to send the ROS message
            send_ros_message()

        # Sleep to control the loop rate
        rate.sleep()


if __name__ == '__main__':
    try:
        # Initialize ROS node
        rospy.init_node('pure_pursuit_control_node')

        # Create a subscriber for the pose topic
        rospy.Subscriber('/actual_pose', PoseStamped, amcl_callback)

        # Create a publisher for the twist topic
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Create a subscriber for the local path topic
        rospy.Subscriber('/local_path', Path, local_path_callback)

        # Set the rate at which to run the main loop
        rate = rospy.Rate(100)

        while not rospy.is_shutdown():
            if path_ready_flag:
                send_ros_message()

            rate.sleep()

    except rospy.ROSInterruptException:
        pass        