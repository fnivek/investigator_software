#!/usr/bin/env python

import rospy
from hardware_interface.srv import CommSrv, CommSrvRequest
from hardware_interface.msg import MotorPWM
from struct import pack

# pack up pwm signals
def pack_pwm(left_pwm, right_pwm):
    # Forward = 0
    # Reverse = 1
    l_dir = 0                       
    if left_pwm < 0: l_dir = 1      
    r_dir = 0
    if r_dir < 0: r_dir = 1

    # Set up direction for packing
    direction = l_dir | (r_dir << 1)

    return pack('<Bff', direction, abs(left_pwm), abs(right_pwm))

# Print whatever was read in hex
def print_hex_cb(data):
    print [format(ord(i), "#04x") for i in data]


class Register:
    # RnW - True if read only False if Write only
    # address - mapping of the register
    # size - number of bytes to write or number of bytes to read
    # callback - on read this function is called to handle the data
    # pack - on write this funciton packs the data
    def __init__(self, RnW, address, size, pack = None, callback = None):
        self.RnW = RnW              
        self.address = address
        self.size = size
        self.callback = callback
        self.pack = pack

        self.comms_proxy = rospy.ServiceProxy('comms', CommSrv)

    def read(self):
        # Check conditions
        if not self.RnW:
            rospy.logerr('Attempting to read MSP430 Register ' + str(self.addr) + ' which is write only')

        # Read the register
        read = self.comms_proxy(CommSrvRequest(
                RnW =           True,
                destination =   CommSrvRequest.MSP430,
                addr =          self.address,
                data =          [0x55] * self.size))

        # Perform callback if nesisary
        if self.callback is not None:
            self.callback(read.data)

    def write(self, *data):
        # Check conditions
        if self.RnW:
            rospy.logerr('Attempting to write MSP430 Register ' + str(self.addr) + ' which is read only')
        if self.pack is None:
            rospy.logerr('MSP430 Register ' + str(self.addr) + ' does not have a pack method')
            return

        # pack and send the data
        packed_data = self.pack(*data)
        self.comms_proxy(CommSrvRequest(
                RnW =           False,
                destination =   CommSrvRequest.MSP430,
                addr =          self.address,
                data =          packed_data))

class Msp430Interface:
    registers = {'SONAR'            : Register(True,    1,  8,  callback =  print_hex_cb),
                 'ENCODERS_SPEED'   : Register(True,    2,  5,  callback =  print_hex_cb),
                 'ENCODERS_POSITION': Register(True,    3,  8,  callback =  print_hex_cb),
                 'MOTORS'           : Register(False,   4,  9,  pack =      pack_pwm),
                 'SET_STATUS'       : Register(False,   5,  1),
                 'GET_STATUS'       : Register(True,    6,  1,  callback =  print_hex_cb)}

    def __init__(self):
        rospy.init_node('msp430_interface', anonymous=False)

        # Comms service proxy
        # Wait for service to come up
        rospy.loginfo('Waiting for comms...')
        while not rospy.is_shutdown():
            try:
                rospy.wait_for_service('comms', 1)
                break
            except Exception as e:
                rospy.logerr('Timeout waiting for comms service')

        
        rospy.loginfo('comms service up')

        # Publishers

        # Timers
        rospy.Timer(rospy.Duration(1./25), self.read_values)        

        # Subscribers
        self.left_wheel_pwm_sub = rospy.Subscriber("motor_pwm", MotorPWM, self.pwm_cb, queue_size = 1)
    
    # Reads in all read only registers
    def read_values(self, event):
        for reg in filter(lambda x: x.RnW == True, Msp430Interface.registers.itervalues()):
            reg.read()

    def pwm_cb(self, pwm):
        Msp430Interface.registers['MOTORS'].write(pwm.left_pwm, pwm.right_pwm)

if __name__ == '__main__':
    msp = Msp430Interface()
    rospy.spin()
