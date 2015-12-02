#!/usr/bin/env python

import rospy
import Leap
import numpy as np
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from collections import deque

MAX_LINEAR_VEL = 0.7
MAX_ANGULAR_VEL = 1.
MOVING_AVG_LENGTH = 3

def XYZArray(r):
    return np.array([r[0], r[1], r[2]])

class LeapController:
    def __init__(self):
        rospy.init_node('leap_controller', anonymous=False)
        self.iface = Leap.Controller()

        # TODO: Add timeout
        rospy.loginfo('Waiting to connect to leap motion')
        while not self.iface.is_connected:
            pass

        rospy.loginfo('Connected to Leap motion')

        self.pitch_buf = deque([], maxlen = MOVING_AVG_LENGTH)
        self.roll_buf = deque([], maxlen = MOVING_AVG_LENGTH)

        # Publishers
        self.pwm_pub = rospy.Publisher('/motion_control/twist', Twist, queue_size = 1)

        self.update_rate = rospy.Rate(10)

    def linear_map(self, pitch, roll):
        # Max pitch = 0.4, Min pitch = -0.4, 0 = 0
        # Max roll = 0.3, Min pitch = -0.3, 0 = 0

        pitch /= 0.7
        roll /= 0.7

        return self.clip_values(pitch, roll)

    def quadratic_map(self, pitch, roll):
        pitch, roll = self.linear_map(pitch, roll)
        sign_p = np.sign(pitch)
        sign_r = np.sign(roll)
        return sign_p * (pitch**2), sign_r * (roll**2)

    def clip_values(self, pitch, roll):
        if pitch > 1:
            pitch = 1
        elif pitch < -1:
            pitch = -1

        if roll > 1:
            roll = 1
        elif roll < -1:
            roll = -1

        return pitch, roll

    def run(self):
        while not rospy.is_shutdown():
            hands = self.iface.frame().hands
            if len(hands) is 1:
                palm_normal = XYZArray(hands.leftmost.palm_normal)

                y_hat = np.array([0, 1, 0])


                # Get and normalize pitch of hand to [1,-1]
                palm_yz = np.copy(palm_normal)
                palm_yz[0] = 0
                pitch = np.arccos(np.dot(palm_yz, y_hat) / np.linalg.norm(palm_yz))
                if pitch < np.pi / 2:
                    continue
                pitch -= np.pi / 2
                pitch *= np.sign(palm_normal[2])
                pitch /= (-np.pi / 2)
                pitch += np.sign(palm_normal[2])

                # Get and normalize roll of hand to [1,-1]
                palm_xy = palm_normal
                palm_xy[2] = 0
                roll = np.arccos(np.dot(palm_xy, y_hat) / np.linalg.norm(palm_xy))
                if roll < np.pi / 2:
                    continue
                roll -= np.pi / 2
                roll *= -1 * np.sign(palm_normal[0])
                roll /= (-np.pi / 2)
                roll += -1 * np.sign(palm_normal[0])
                roll *= -1

                # Map the values
                pitch, roll = self.quadratic_map(pitch, roll)
                #print 'pitch: %f\t\t roll: %f' % (pitch, roll)

                # Windowed avg
                self.pitch_buf.append(pitch)
                self.roll_buf.append(roll)
                pitch = sum(self.pitch_buf) / len(self.pitch_buf)
                roll = sum(self.roll_buf) / len(self.roll_buf)

                # Send out the twist
                linear = Vector3()
                linear.x = pitch * MAX_LINEAR_VEL

                angular = Vector3()
                angular.z = roll * MAX_ANGULAR_VEL

                self.pwm_pub.publish(Twist(
                    linear = linear, 
                    angular = angular))
            else:
                # Publish 0 Twist
                self.pitch_buf.append(0)
                self.roll_buf.append(0)
                self.pwm_pub.publish(Twist())

            self.update_rate.sleep()


if __name__ == '__main__':
    leap_controller = LeapController()
    leap_controller.run()