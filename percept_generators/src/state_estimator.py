#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from collections import deque
import tf

# State esitimator takes in state estimations from several estimators in base_link
#   Such as encoders and the IMU and combines them into one global state estimation
#   which is published on the state topic and as a tf transform between the world
#   frame and base_link
#
#   Work in progress ....

class StateEstimator:
    def __init__(self):
        rospy.init_node('state_estimator', anonymous = False)

        # Publishers
        self.state_pub = rospy.Publisher('state', Odometry, queue_size = 1)

        # Tf transforms
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_listener = tf.TransformListener()

        # Deque for holding changes in state
        #self.state_changes = deque()
        self.state_trans = [0, 0, 0]
        self.state_rot = [0, 0, 0, 1]

        # Subscribers
        self.encoder_sub = rospy.Subscriber('encoder_state', Odometry, self.encoder_state_cb, queue_size = 1)

        # Timers
        #rospy.Timer(rospy.Duration(1./25.), self.update)

    def update(self, event):
        # This is where states from encoders, IMU, or anything else would be fused
        #   into one world model
        pass


    def encoder_state_cb(self, msg):
        # This function will temporarly just convert to world frame and publish
        #   odometry and a tf transform
        temp_pose = PoseStamped(
            header = msg.header,
            pose = msg.pose.pose)
        # Reverse time far enough to ensure good tf data
        temp_pose.header.stamp -= rospy.Duration(1.1/25.)

        try:
            msg.pose.pose = (self.tf_listener.transformPose('/world', temp_pose)).pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('TF error: %s' % e)
            return
            # TODO: make sure we don't reset our position on an error

        # Update header to reflect transform
        msg.header.frame_id = '/world'
        self.state_pub.publish(msg)

        self.state_trans = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
            ]

        self.state_rot = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
            ]

        self.tf_broadcaster.sendTransform(self.state_trans,
                                  self.state_rot,
                                  rospy.Time.now(),
                                  'base_link',
                                  'world')

        #print '[dx\', dy\']: [%f, %f]' % (dx, dy)

if __name__ == '__main__':
    state_estimator = StateEstimator()
    rospy.spin()