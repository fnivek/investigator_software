#!/usr/bin/env python

import rospy
import Leap
import numpy as np
from geometry_msgs.msg import Vector3, Pose, Twist, PoseStamped
from collections import deque
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import tf
from motion_control.srv import WaypointCmd, WaypointCmdResponse, WaypointCmdRequest

MAX_LINEAR_VEL = 0.7
MAX_ANGULAR_VEL = 1.
MOVING_AVG_LENGTH = 3

def clip_angles(angle):
    return ((angle + np.pi) % (2 * np.pi)) - np.pi

def XYZArray(r):
    return np.array([r[0], r[1], r[2]])

class LeapController:
    def __init__(self):
        rospy.init_node('leap_controller', anonymous=False)

        self.update_hz = 10

        self.mode = 'rc'
        self.switch_in_progress = False

        self.iface = Leap.Controller()

        # TODO: Add timeout
        rospy.loginfo('Waiting to connect to leap motion')
        while not self.iface.is_connected:
            pass

        rospy.loginfo('Connected to Leap motion')

        self.iface.enable_gesture(Leap.Gesture.TYPE_SWIPE)
        #self.iface.enable_gesture(Leap.Gesture.TYPE_KEY_TAP)
        self.iface.enable_gesture(Leap.Gesture.TYPE_CIRCLE)
        self.iface.config.set("Gesture.Circle.MinRadius", 30.0)
        self.iface.config.save()

        self.pitch_buf = deque([], maxlen = MOVING_AVG_LENGTH)
        self.roll_buf = deque([], maxlen = MOVING_AVG_LENGTH)

        # Tf listener
        self.tf_listener = tf.TransformListener()

        # Publishers
        self.pwm_pub = rospy.Publisher('/motion_control/twist', Twist, queue_size = 1)
        self.waypoint_pub = rospy.Publisher('/motion_control/waypoint', Pose, queue_size = 1)
        self.waypoint_viz_pub = rospy.Publisher('waypoint_viz', Marker, queue_size = 1)

        # Services
        self.waypoint_cmd_proxy = rospy.ServiceProxy('/motion_control/waypoint_cmd', WaypointCmd)
        
        # Set up waypoint marker
        self.placing_color = ColorRGBA(1, 0, 0.597, 0.5)
        self.set_color = ColorRGBA(0.3984, 1, 0, 0.5)
        self.waiting_color = ColorRGBA(1, 1, 0, 0.5)
        self.confirm_color = ColorRGBA(0, 1, 0.797, 0.5)

        self.waypoint_marker = Marker()
        self.waypoint_marker.header.stamp = rospy.Time.now()
        self.waypoint_marker.header.frame_id = 'world'
        self.waypoint_marker.ns = 'user_interface'
        self.waypoint_marker.id = 0
        self.waypoint_marker.type = Marker.ARROW
        self.waypoint_marker.action = Marker.ADD
        self.waypoint_marker.pose = Pose()
        self.waypoint_marker.scale.x = 0.2
        self.waypoint_marker.scale.y = 0.05
        self.waypoint_marker.scale.z = 0.05
        self.waypoint_marker.color = self.set_color
        #self.waypoint_marker.frame_locked = True

        self.waypoint_turn_rate = 50 * (np.pi / (180 * self.update_hz))
        self.cw_dead_zone = -0.45
        self.ccw_dead_zone = 0.3
        self.waypoint_heading = 0

        self.waypoint_pose = Pose()
        self.place_time = rospy.Time.now()                  # Time of last placed waypoint
        self.wait_time_waypoint = rospy.Duration(0.5)    # How long after droping a waypoint to send cmd
        self.waypoint_placeable_radius = 6.0

        self.waypoint_state = 'idle'

        self.update_rate = rospy.Rate(self.update_hz)

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

    def zero_all(self):
        self.pwm_pub.publish(Twist())


    def rc_cmd(self, hand):
        palm_normal = XYZArray(hand.palm_normal)

        y_hat = np.array([0, 1, 0])


        # Get and normalize pitch of hand to [1,-1]
        palm_yz = np.copy(palm_normal)
        palm_yz[0] = 0
        pitch = np.arccos(np.dot(palm_yz, y_hat) / np.linalg.norm(palm_yz))
        if pitch < np.pi / 2:
            return
        pitch -= np.pi / 2
        pitch *= np.sign(palm_normal[2])
        pitch /= (-np.pi / 2)
        pitch += np.sign(palm_normal[2])

        # Get and normalize roll of hand to [1,-1]
        palm_xy = palm_normal
        palm_xy[2] = 0
        roll = np.arccos(np.dot(palm_xy, y_hat) / np.linalg.norm(palm_xy))
        if roll < np.pi / 2:
            return
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

    def waypoint_cmd(self, hand, gestures):
        pos = np.array([-(hand.palm_position.z / 60.0)
                        -(hand.palm_position.x / 60.0)])

        r = np.linalg.norm(pos)

        if r > self.waypoint_placeable_radius:
            self.mode = 'idle'
            return


        if hand.pinch_strength > 0.9:
            self.waypoint_state = 'placing'

            self.waypoint_pose = Pose()
            self.waypoint_pose.position.x = -(hand.palm_position.z / 30.0)
            self.waypoint_pose.position.y = -(hand.palm_position.x / 30.0)

            pitch = hand.palm_normal.z

            if pitch > self.ccw_dead_zone or pitch < self.cw_dead_zone:
                self.waypoint_heading = clip_angles(self.waypoint_heading + np.sign(pitch) * self.waypoint_turn_rate)

            quat = tf.transformations.quaternion_from_euler(0, 0, self.waypoint_heading)
            self.waypoint_pose.orientation.x = quat[0]
            self.waypoint_pose.orientation.y = quat[1]
            self.waypoint_pose.orientation.z = quat[2]
            self.waypoint_pose.orientation.w = quat[3]

            self.convert_pose('/base_link', '/world')

        # State transition from placing to waiting
        elif self.waypoint_state == 'placing':
            self.waypoint_state = 'waiting'
            # Done placing point start timer to see 
            self.place_time = rospy.Time.now()

        # Wait self.wait_time_waypoint seconds
        elif self.waypoint_state == 'waiting':
            if rospy.Time.now() - self.place_time > self.wait_time_waypoint:
                self.waypoint_state = 'confirm'
                self.waiting_to_set_waypoint = False

        elif self.waypoint_state == 'confirm':
            swipe = False
            for gesture in gestures:
                if gesture.type == Leap.Gesture.TYPE_SWIPE:
                    swipe = True

            if swipe:
                self.waypoint_pub.publish(self.waypoint_pose)
                self.waypoint_state = 'idle'

    def convert_pose(self, from_frame, to_frame):
        pt = PoseStamped()
        pt.pose = self.waypoint_pose
        pt.header.frame_id = from_frame
        pt.header.stamp = rospy.Time()
        try:
            pt = self.tf_listener.transformPose(to_frame, pt)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr('TF error: %s' % e)
            return False

        self.waypoint_pose = pt.pose        
        return True

    def draw_waypoint(self):
        if self.waypoint_state is 'idle':
            self.waypoint_marker.color = self.set_color
        elif self.waypoint_state is 'placing':
            self.waypoint_marker.color = self.placing_color
        elif self.waypoint_state is 'waiting':
            self.waypoint_marker.color = self.waiting_color
        elif self.waypoint_state is 'confirm':
            self.waypoint_marker.color = self.confirm_color

        self.waypoint_marker.pose = self.waypoint_pose
        self.waypoint_marker.header.stamp = rospy.Time()
        self.waypoint_viz_pub.publish(self.waypoint_marker)


    def switch_mode(self):
        if self.mode == 'rc':
            self.mode = 'waypoint'
            self.waypoint_cmd_proxy(True)
        else:
            self.mode = 'rc'
            self.waypoint_cmd_proxy(False)

        self.zero_all()

        rospy.loginfo('Switch to %s mode' % self.mode)

    def run(self):
        while not rospy.is_shutdown():
            hands = self.iface.frame().hands
            gestures = self.iface.frame().gestures()

            if self.mode == 'rc':
                if len(hands) is 1:
                    self.rc_cmd(hands.leftmost)
                else:
                    # Publish 0 Twist
                    self.pitch_buf.append(0)
                    self.roll_buf.append(0)
                    self.pwm_pub.publish(Twist())
            else:
                if len(hands) is 1:
                    self.waypoint_cmd(hands.leftmost, gestures)
                else:
                    self.waypoint_state = 'idle'

                """
                if self.waypoint_state != 'confirm':
                    for gesture in gestures:
                        if gesture.type == Leap.Gesture.TYPE_SWIPE:
                            print 'Clear'
                            self.waypoint_state = 'idle'
                            self.waypoint_cmd_proxy(True)
                            self.waypoint_pose = Pose()
                            self.convert_pose('/base_link', '/world')
                """

                self.draw_waypoint()


            # mode switch logic
            circle_detected = False
            if len(hands) is 2:
                circle_detected = False
                if len(gestures) != 0:
                    for gesture in gestures:
                        if gesture.type == Leap.Gesture.TYPE_CIRCLE:
                            # Circle gesture detected
                            circle_detected = True
            if circle_detected:
                self.switch_in_progress = True
            elif self.switch_in_progress:
                self.switch_in_progress = False
                self.switch_mode()

            self.update_rate.sleep()


if __name__ == '__main__':
    leap_controller = LeapController()
    leap_controller.run()