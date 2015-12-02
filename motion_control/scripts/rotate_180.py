#!/usr/bin/env python

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospy
import tf
import numpy

rospy.init_node('rotate_180', anonymous = False)

twist_pub = rospy.Publisher('/motion_control/twist', Twist, queue_size = 1)

error = 1e9
first = True
final_heading = 0


def feedback_cb(msg):
    global first
    global error
    global final_heading

    quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
                ]

    trash, trash, current_heading = tf.transformations.euler_from_quaternion(quat)

    current_heading = current_heading % (2 * numpy.pi)

    if first:
        final_heading = (current_heading + numpy.pi) % (2 * numpy.pi)
        first = False

        print 'At: ', current_heading, '\tMoving to: ', final_heading

    #print 'current ', current_heading

    error = abs(current_heading - final_heading)

rospy.Subscriber('/percepts/state', Odometry, feedback_cb, queue_size = 1)

t = Twist()
t.angular.z = 3

while error > (5 * numpy.pi / 180):
    twist_pub.publish(t)

twist_pub.publish(Twist())
