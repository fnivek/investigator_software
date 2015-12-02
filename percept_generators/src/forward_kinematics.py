#!/usr/bin/env python

# This file does forward kinematics for the data from the encoders
#	ICC - instantaneous center of curvature
#	R - distance of ICC to the center of wheel base
#	w - anular velocity about ICC
import rospy
import numpy as np
import tf
from hardware_interface.msg import EncoderSpeed
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3, PoseWithCovariance, Pose, TwistWithCovariance, Twist, Point, Quaternion

SAME_FLOAT = 0.001

class node:
	def __init__(self):
		self.last_time = 0
		self.last_R = 0
		self.last_w = 0
		self.wheel_base = rospy.get_param('/wheel_base', 0.5) # 0.381 m = 15 in
		self.wheel_radius = rospy.get_param('/wheel_radius', 0.097)

		self.state_pub = rospy.Publisher('encoder_state', Odometry, queue_size = 1)
		
		self.motor_vel_sub = rospy.Subscriber('/hardware_interface/encoder_speed', EncoderSpeed, self.vels_cb, queue_size = 1)

	#Callback for motor angular velocities
	# 
	# Variables:
	#	wr := right wheel angular velocity (rad/s)
	#	wl := left wheel angular velocity (rad/s)
	#	r_wheel := radius of the wheel (m)
	#	Vr := velocity of right wheel along the ground (m/s)
	#	Vl := velocity of left wheel along the ground (m/s)
	#	Vx := Linear velocity of robot (m/s)
	#	w := angular velocity of robot about instantaneous center of curvature (ICC) (rad/s)
	#	L := length between the wheels (m)
	#
	# Equation:
	#	Vr = wr * r_wheel
	#	Vl = wl * r_wheel
	#	Vx = (Vr + Vl) / 2
	#	w = (Vr - Vl) / L
	# ------------------------
	#	Vr = Vx + L * w / 2
	#	Vl = Vx - L * w / 2
	# ------------------------
	#	wr = (1 / r_wheel) * (Vx + L * w / 2)
	# 	wl = (1 / r_wheel) * (Vx - L * w / 2)
	#
	# Use avg of last R and last w with current
	# multiply by the time between cb's
	def vels_cb(self, msg):

		# Time step
		now = rospy.get_time()
		time_step = now - self.last_time
		self.last_time = now

		#print 'wheel_ang_vels:', msg.data
		# w * r = V (m/s)
		Vl = msg.left_motor * self.wheel_radius
		Vr = msg.right_motor * self.wheel_radius
		Vx = (Vr + Vl) / 2
		w = (Vr - Vl) / self.wheel_base

		dpos_vec = np.matrix([[0.0], [0.0]])
		dtheta = 0

		R = 0
		R_avg = 0
		w_avg = 0

		if(abs(Vl - Vr) < SAME_FLOAT):
			dpos_vec[0] = Vx * time_step  
		else:
			# Instantanious values
			R = self.wheel_base * Vx / (Vr - Vl)

			R_avg = (R + self.last_R) / 2
			w_avg = (w + self.last_w) / 2

			#print '[Ravg, wavg]: [%f, %f]' % (R_avg, w_avg)

			self.last_R = R
			self.last_w = w

			# Kinematic equations assuming no slip
			dtheta = time_step * w_avg			# Angular rads changed along ICC, this is not the change in the reference frame
			dpos_vec[0] = R_avg * np.sin(dtheta)
			dpos_vec[1] = R_avg * (1 - np.cos(dtheta))
			#print dpos_vec
			#print '[dx, dy, d_theta]: [%f, %f, %f]' % (dpos_vec.item(0), dpos_vec.item(1), dtheta)

		quat = tf.transformations.quaternion_from_euler(0, 0, dtheta)

		#test = np.linalg.norm(dpos_vec)
		#if test > 5:
		#	print 'Time step:', time_step
		#	print msg
		#	print 'Jump: ', test
		#	print 'Ravg: ', R_avg
		#	print 'Wavg: ', w_avg

		# Publish the odom estimate
		self.state_pub.publish(Odometry(
			header = Header(
				stamp = rospy.get_rostime(),
				frame_id = 'base_link'),

			child_frame_id = 'base_link',

			pose = PoseWithCovariance(
				pose = Pose(
					position = Point(
						x = dpos_vec[0],
						y = dpos_vec[1]),
					orientation = Quaternion(
						x = quat[0],
						y = quat[1],
						z = quat[2],
						w = quat[3]))),

			twist = TwistWithCovariance(
				twist = Twist(
					linear = Vector3(
						x = Vx),
					angular = Vector3(
						z = w)))))

		#print '[dx\', dy\']: [%f, %f]' % (dx, dy)

def main():
	# Ros initilization
	#Init ros
	rospy.init_node('forward_kinematics', anonymous=False)

	this_node = node()

	rospy.spin()

if __name__ == '__main__':
	main()