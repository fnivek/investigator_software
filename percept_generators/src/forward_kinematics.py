#!/usr/bin/env python

# This file does forward kinematics for the data from the encoders
#	ICC - instantaneous center of curvature
#	R - distance of ICC to the center of wheel base
#	w - anular velocity about ICC
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
import tf
from hardware_interface.msg import EncoderSpeed

class node:
	def __init__(self):
		self.listener = tf.TransformListener()
		self.last_time = 0
		self.last_R = 0
		self.last_w = 0
		self.wheel_base = rospy.get_param('/wheel_base', 0.3048) # 0.3048 m = 12 in
		self.wheel_radius = rospy.get_param('/wheel_radius', 0.06)

		
		self.motor_vel_sub = rospy.Subscriber('encoder_speed', EncoderSpeed, self.vels_cb, queue_size = 1)

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

		dpos_vec = np.matrix([[0.0], [0.0]])
		dtheta = 0

		if(Vl == Vr):
			dpos_vec[0] = Vl * time_step  
		else:
			# Instantanious values
			R = (self.wheel_base * (Vl + Vr)) / (2 * (Vr - Vl))
			w = (Vr - Vl) / self.wheel_base

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

		# Transform cords
		trans = [0,0,0]	#x y z
		rot = [0,0,0,1] # x y z w
		try:
			(trans, rot) = self.listener.lookupTransform('/world', '/base_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
			rospy.logerr('TF error: %s' % e)
			# TODO: make sure we don't reset our position on an error
		trash, trash, frame_theta = tf.transformations.euler_from_quaternion(rot)
		rotation = np.matrix([[np.cos(frame_theta), -1.0 * np.sin(frame_theta)],
				   [np.sin(frame_theta), np.cos(frame_theta)]]) 
		rotated_dpos_vec = rotation * dpos_vec

		rot = tf.transformations.quaternion_multiply(rot, tf.transformations.quaternion_from_euler(0, 0, dtheta))		
		
		# Set up a tf brodcaster
		br = tf.TransformBroadcaster()
		br.sendTransform((trans[0] + rotated_dpos_vec.item(0), trans[1] + rotated_dpos_vec.item(1), 0),
						 rot,
						 rospy.Time.now(),
						 'base_link',
						 'world')

		#print '[dx\', dy\']: [%f, %f]' % (dx, dy)

def main():
	# Ros initilization
	#Init ros
	rospy.init_node('forward_kinematics', anonymous=False)

	this_node = node()

	rospy.spin()

if __name__ == '__main__':
	main()