#!/usr/bin/env python

import rospy
from motion_control.msg import MotorAngularWheelVelocities
from hardware_interface.msg import EncoderSpeed, MotorPWM
from std_msgs.msg import Float64
from collections import deque
import numpy as np

NUM_MOTORS = 2
LEFT = 0
RIGHT = 1
FCPU = 2.44e6
ENCODER_CLKS_TO_RADS_SEC = 2 * np.pi * 8 / (898 * FCPU)

class Motor:
    NUM_OUTPUTS = 2
    NUM_FEEDBACKS = 3
    # Coefs should be a numpy array where the first NUM_OUTPUTS entries are the coefficents of
    #   the outputs and the next NUM_FEEDBACKS entries should be the feedback coefficents
    #   i.e. out[n] = c0*out[n-1] + c1 * out[n-2] + c2 * feedback[n] + c3 * feedback[n-1] + c4 * feedback[n-2]
    def __init__(self, coefs):
        self.feedbacks = deque([0 for i in xrange(Motor.NUM_FEEDBACKS)], maxlen = Motor.NUM_FEEDBACKS)
        self.outputs = deque([0 for i in xrange(Motor.NUM_OUTPUTS)], maxlen = Motor.NUM_OUTPUTS)
        self.desired_speed = 0
        self.coef_vector = coefs

    def set_last_speed(self, speed):
        self.feedbacks.append(speed * ENCODER_CLKS_TO_RADS_SEC)

    def set_desired_speed(self, speed):
        # TODO: Clip values to a min and max
        self.desired_speed = speed

    # out[n] = c0*out[n-1] + c1 * out[n-2] + c2 * feedback[n] + c3 * feedback[n-1] + c4 * feedback[n-2]
    def calculate_output(self):
        values = np.concatenate(([self.outputs[Motor.NUM_OUTPUTS - 1 - i] for i in xrange(Motor.NUM_OUTPUTS)], 
                                 [self.feedbacks[Motor.NUM_FEEDBACKS - 1 - i] for i in xrange(Motor.NUM_FEEDBACKS)]),
                                    axis = 1)
        # Calculate output voltage
        out = np.dot(values, self.coef_vector)
        self.outputs.append(out)

        # Calculate pulse width from voltage
        # TODO: remove magic numbers
        if out > 6: out = 6
        if out < -6: out = -6

        return out * 2000. / 6.

class WheelSpeedController:
    def __init__(self):
        rospy.init_node('wheel_speed_controller', anonymous = False)
        self.last_time = rospy.get_time()

        coefs = [np.array([0.9132, 0.08676, 0.5932, 0.339, -0.2542]), np.array([0.9132, 0.08676, 0.5932, 0.339, -0.2542])] # 5.932, 3.39, -2.542
        self.motors = [Motor(coefs[i]) for i in xrange(NUM_MOTORS)]

        # Publishers
        self.motor_pwm_pub = rospy.Publisher('motor_pwm', MotorPWM, queue_size = 1)
        self.debug_pub = rospy.Publisher('debug', Float64, queue_size = 1)

        # Subscribers
        self.feedback_sub = rospy.Subscriber('encoder_speed', EncoderSpeed, self.feedback_cb, queue_size = 1)
        self.desired_sub = rospy.Subscriber('motor_angular_wheel_velocities', MotorAngularWheelVelocities, self.command_cb, queue_size = 1)

    def command_cb(self, desired):
        # Update desired
        self.motors[LEFT].set_desired_speed(desired.left_angular_velocity)
        self.motors[RIGHT].set_desired_speed(desired.right_angular_velocity)

    # Implement lead controller
    def feedback_cb(self, speed):
        new = rospy.get_time()
        freq = (new - self.last_time) ** -1
        self.last_time = new
        rospy.loginfo('Freq: ' + str(freq))
        # Update last speed
        self.motors[LEFT].set_last_speed(speed.left_motor)
        self.motors[RIGHT].set_last_speed(speed.right_motor)

        self.debug_pub.publish(self.motors[LEFT].feedbacks[Motor.NUM_FEEDBACKS - 1])

        # Calculate next value
        self.motor_pwm_pub.publish(MotorPWM(
            left_pwm = self.motors[LEFT].calculate_output(),
            right_pwm = self.motors[RIGHT].calculate_output()))



if __name__ == '__main__':
    wsc = WheelSpeedController()
    rospy.spin()