#!/usr/bin/env python

import rospy
from motion_control.msg import MotorAngularWheelVelocities
from hardware_interface.msg import EncoderSpeed, MotorPWM
from std_msgs.msg import Float64
from collections import deque
import threading
import numpy as np

NUM_MOTORS = 2
LEFT = 0
RIGHT = 1
FCPU = 2.44e6
ENCODER_CLKS_TO_RADS_SEC = 2 * np.pi * FCPU / (898)

class Motor:
    NUM_OUTPUTS = 2
    NUM_ERRORS = 3
    MIN_SPEED = 0.6 #Rad/s
    # Coefs should be a numpy array where the first NUM_OUTPUTS entries are the coefficents of
    #   the outputs and the next NUM_ERRORS entries should be the feedback coefficents
    #   i.e. out[n] = c0*out[n-1] + c1 * out[n-2] + c2 * feedback[n] + c3 * feedback[n-1] + c4 * feedback[n-2]
    def __init__(self, coefs):
        self.errors = deque([0 for i in xrange(Motor.NUM_ERRORS)], maxlen = Motor.NUM_ERRORS)
        self.feedbacks = []
        self.last_avg_feedback = 0
        self.outputs = deque([0 for i in xrange(Motor.NUM_OUTPUTS)], maxlen = Motor.NUM_OUTPUTS)
        self.desired_speed = 0
        self.coef_vector = coefs
        self.lock = threading.Lock()

    def set_last_speed(self, speed):
        with self.lock:
            if speed == 0xFFFF:
                self.feedbacks.append(0)
            else:
                self.feedbacks.append((speed**-1) * ENCODER_CLKS_TO_RADS_SEC)

    def calculate_error(self):
        # Average all feedbacks
        with self.lock:
            if len(self.feedbacks) != 0:
                self.last_avg_feedback = sum(self.feedbacks) / len(self.feedbacks)

            self.feedbacks = []

        # Append next errror
        self.errors.append(self.desired_speed - self.last_avg_feedback)

    def set_desired_speed(self, speed):
        if abs(speed) >= Motor.MIN_SPEED:
            self.desired_speed = speed
        else:
            self.desired_speed = 0

    # out[n] = c0*out[n-1] + c1 * out[n-2] + c2 * feedback[n] + c3 * feedback[n-1] + c4 * feedback[n-2]
    def calculate_output(self):
        values = np.concatenate(([self.outputs[Motor.NUM_OUTPUTS - 1 - i] for i in xrange(Motor.NUM_OUTPUTS)], 
                                 [self.errors[Motor.NUM_ERRORS - 1 - i] for i in xrange(Motor.NUM_ERRORS)]),
                                    axis = 1)

        # Calculate output voltage
        out = np.dot(values, self.coef_vector)

        # Clip
        if out > 6: out = 6
        if out < -6: out = -6

        self.outputs.append(out)

        # Calculate pulse width from voltage
        # TODO: remove magic numbers
        return out * 2000. / 6.

class WheelSpeedController:
    def __init__(self):
        rospy.init_node('wheel_speed_controller', anonymous = False)

        coefs = [np.array([0.7634, 0.2366, 0.34105, 0.21491, -0.12615]), np.array([0.7634, 0.2366, 0.34105, 0.21491, -0.12615])] # 5.932, 3.39, -2.542
        self.motors = [Motor(coefs[i]) for i in xrange(NUM_MOTORS)]

        # Publishers
        self.motor_pwm_pub = rospy.Publisher('motor_pwm', MotorPWM, queue_size = 1)

        # Subscribers
        self.feedback_sub = rospy.Subscriber('encoder_speed', EncoderSpeed, self.feedback_cb, queue_size = 1)
        self.desired_sub = rospy.Subscriber('motor_angular_wheel_velocities', MotorAngularWheelVelocities, self.command_cb, queue_size = 1)

        rospy.Timer(rospy.Duration(1./25.), self.controller)

    def command_cb(self, desired):
        # Update desired
        self.motors[LEFT].set_desired_speed(desired.left_angular_velocity)
        self.motors[RIGHT].set_desired_speed(desired.right_angular_velocity)

    # Implement lead controller
    def feedback_cb(self, speed):
        # Update last speed
        self.motors[LEFT].set_last_speed(speed.left_motor)
        self.motors[RIGHT].set_last_speed(speed.right_motor)

    def controller(self, event):
        # Average the previous feedbacks
        for motor in self.motors:
            motor.calculate_error()

        # Calculate next value
        self.motor_pwm_pub.publish(MotorPWM(
            left_pwm = self.motors[LEFT].calculate_output(),
            right_pwm = self.motors[RIGHT].calculate_output()))



if __name__ == '__main__':
    wsc = WheelSpeedController()
    rospy.spin()