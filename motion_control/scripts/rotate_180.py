#!/usr/bin/env python

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import rospy
import tf
import numpy

rospy.init_node('rotate_180', anonymous = False)

twist_pub = rospy.Publisher('twist', Twist, queue_size = 1)

last_heading = 0
delta_heading = 0
first = True
Kv = 1.0


def feedback_cb(msg):
    global first
    global delta_heading
    global last_heading

    quat = [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
                ]

    trash, trash, current_heading = tf.transformations.euler_from_quaternion(quat)

    current_heading = current_heading % (2 * numpy.pi)

    if first:
        last_heading = current_heading
        first = False

        print 'At: ', current_heading, '\tMoving 2 pi rads'

    print 'current ', current_heading
    print 'last ', last_heading
    print 'diff ', current_heading - last_heading

    delta_heading += current_heading - last_heading

    print 'Cumulative: ', delta_heading

    last_heading = current_heading

    error = numpy.pi - delta_heading

    t = Twist()
    t.angular.z = Kv * error


    twist_pub.publish(t)

rospy.Subscriber('state', Odometry, feedback_cb, queue_size = 1)

rospy.spin()
