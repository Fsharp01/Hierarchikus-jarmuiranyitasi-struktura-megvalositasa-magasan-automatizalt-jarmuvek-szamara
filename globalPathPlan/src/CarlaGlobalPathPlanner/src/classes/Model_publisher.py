#!/usr/bin/env python3

"""

Lexus Modell publiser
"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import tf
import pathlib
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class PositionRefresher():
    


        def __init__(self):
            """
            Self parameters
            """
            self.position_x = -3.421
            self.position_y = -8.410
            self.yaw=1.5
            self.X_dot = 0
            self.Y_dot = 0
            self.psi_dot = 0
            self.beta_dot = 0
            self.speed=0
            self.theta=0

            self.br = tf.TransformBroadcaster()
            self.dt=rospy.Rate(100)
            rospy.logwarn("Init modell")
            
            """
            ROS pubs
            """
            self.actual_pose_pub= rospy.Publisher('/actual_pose',PoseStamped,queue_size=10)

            """
            ROS subs
            """            
            self.control_sub = rospy.Subscriber('/cmd_vel', Twist, self.control_inputs)

        def control_inputs(self,msg):
            self.speed=msg.linear.x
            self.theta=msg.angular.z

        
        def kinematic_vehicle_model(self,V, psi, beta, delta_f, delta_r, l_f, l_r):
            """
            Calculate the rates of change of position and orientation for a vehicle based on a kinematic model.
        
            Parameters:
            V       : Velocity of the vehicle.
            psi     : Yaw angle (orientation angle of vehicle with respect to global X axis).
            beta    : Vehicle slip angle.
            delta_f : Front steering angle.
            delta_r : Rear steering angle.
            l_f     : Distance from vehicle's center of gravity to the front axle.
            l_r     : Distance from vehicle's center of gravity to the rear axle.
        
            Returns:
            A tuple containing rates of change of X, Y, and yaw angle.
            """
            # Equations based on the kinematic model
            self.X_dot = V * math.cos(psi + beta)
            self.Y_dot = V * math.sin(psi + beta)
            self.psi_dot = (V / (l_f + l_r)) * math.cos(beta) * (math.tan(delta_f) - math.tan(delta_r))
            self.beta_dot = math.atan((l_f * math.tan(delta_f) + l_r * math.tan(delta_r)) / (l_f + l_r))

             # Integrate velocities to update position and orientation
            self.position_x += self.X_dot * 0.01
            self.position_y += self.Y_dot * 0.01
            self.yaw += self.psi_dot * 0.01

            self.br.sendTransform((self.position_x, self.position_y, 0),
                                  tf.transformations.quaternion_from_euler(0, 0, self.yaw),
                                  rospy.Time.now(),
                                  "base_link",
                                  "map")   
            #rospy.logwarn("kinematics updated")       
            pose=PoseStamped()
            pose.header.frame_id="map"
            pose.header.stamp=rospy.Time.now()
            pose.pose.position.x=self.position_x
            pose.pose.position.y=self.position_y
            pose.pose.orientation.z=self.yaw
            
            self.actual_pose_pub.publish(pose)


def main():
    """
    Main function
    """

    try:
        rospy.init_node("robot_pose_node", anonymous=True)
        rospy.logwarn("Model Vehicle is spawned")
        robot_pose_node=PositionRefresher()
        while not rospy.is_shutdown():
            robot_pose_node.kinematic_vehicle_model(robot_pose_node.speed,robot_pose_node.yaw,0,robot_pose_node.theta,0,1.395,1.395)
           # rospy.logwarn("Pose update")
        robot_pose_node.dt.sleep()
        rospy.spin()

    except rospy.ROSException:
        rospy.logerr("Error while waiting for info!")
        sys.exit(1)

    finally:
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
    