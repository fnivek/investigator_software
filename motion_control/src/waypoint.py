#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
import numpy as np
import tf

# This file takes in a goal pose in the world frame and 
#   Publishes twist to move the robot towards the goal
#   pose. To do so an algorithim outlined in _paper_name_ (_URL_)
#   was used
#
#   The robot is always active attempting to achieve its latest 
#   command.
#
#   Cordinate System - The cordinates used are polar with the robot as the
#       origin, a ray drawn from the robot passing through the goal position
#       is at an angle of 0 degrees.
#
#   State - a vector composed of the robots state in a polar cordinate
#       System with the robot as the origin
#       r - distance from its goal,
#       theta - angle between goal orientation and vector drawn to goal, and
#       delta - angle between current orientation and vector drawn to goal 
#
#   Algorithim:
#       * Get current location in /world
#       * Calculate state based off of goal position and
#           current location
#       * Calculate control variables
#       * Output to inverse kinematics

def clip_angles(angle):
    return ((angle + np.pi) % (2 * np.pi)) - np.pi

class Waypoint:
    def __init__(self):
        rospy.init_node('waypoint', anonymous = False)

        self.goal_pos = np.array([0,0])
        self.goal_heading = 0

        self.kv = rospy.get_param('kv', 0.5)
        self.k1 = rospy.get_param('k1', 0.5)
        self.k2 = rospy.get_param('k2', 10.)

        # Publishers
        self.twist_pub = rospy.Publisher('twist', Twist, queue_size = 1)

        # Subscribers
        self.goal_sub = rospy.Subscriber('waypoint', Pose, self.goal_cb, queue_size = 1)
        self.feedback_sub = rospy.Subscriber('/percepts/state', Odometry, self.feedback_cb, queue_size = 1)

    def goal_cb(self, msg):
        self.goal_pos = np.array([msg.position.x,
                                     msg.position.y])

        trash, trash, self.goal_heading = tf.transformations.euler_from_quaternion((
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
            ))

    def feedback_cb(self, feedback):
        current_pos = np.array([feedback.pose.pose.position.x,
                                feedback.pose.pose.position.y])

        # Heading is between -pi and pi
        trash, trash, heading = tf.transformations.euler_from_quaternion((
            feedback.pose.pose.orientation.x,
            feedback.pose.pose.orientation.y,
            feedback.pose.pose.orientation.z,
            feedback.pose.pose.orientation.w
            ))

        to_goal = self.goal_pos - current_pos
        # World angle coresponding to 0 angle in the polar cordinate system defined above 
        zero_angle = np.arctan2(to_goal[1], to_goal[0]) # [-pi, pi]
             

        # Calculate state
        r = np.linalg.norm(to_goal)                              # Distance to goal
        theta = clip_angles(self.goal_heading - zero_angle)      # Refer to paper
        delta = clip_angles(heading - zero_angle)                # Refer to paper


        v = 0
        w = 0

        # Calculate control laws
        if r > 0.05:
            v = self.kv * r
            #     -kv    * (   k2   * (delta -    atan(    -k1    * theta)) + (1 + (   k1   / ( 1 + (   k1   * theta)^2 )) * sin(delta)));
            w = -self.kv * (self.k2 * (delta - np.arctan(-self.k1 * theta)) + (1 + (self.k1 / ( 1 + (self.k1 * theta)**2)) * np.sin(delta)))

        # Publish 
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.twist_pub.publish(t)


if __name__ == '__main__':
    wp = Waypoint()
    rospy.spin()
