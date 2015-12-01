#!/usr/bin/env python

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import rospy
import tf
import numpy

rospy.init_node('forward_one_meter', anonymous = False)

twist_pub = rospy.Publisher('twist', Twist, queue_size = 1)

current_pose = numpy.array([0, 0])
final_pose = [1, 0]
first = True
Kv = 0.25


def feedback_cb(msg):
    global first
    global final_pose
    current_pose = numpy.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
    if first:
        quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
                ]

        trash, trash, heading = tf.transformations.euler_from_quaternion(quat)

        final_pose = numpy.array([current_pose[0] + numpy.cos(heading), current_pose[1] + numpy.sin(heading)])
        first = False

        print 'At: ', current_pose, '\tMoving to: ', final_pose


    error = numpy.linalg.norm(final_pose - current_pose)

    t = Twist()
    t.linear.x = Kv * error


    twist_pub.publish(t)

rospy.Subscriber('state', Odometry, feedback_cb, queue_size = 1)

rospy.spin()
