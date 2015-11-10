#!/usr/bin/env python

import rospy
from hardware_interface.srv import CommSrv, CommSrvRequest

class Register:
    # RnW - True if read only False if Write only
    # address - mapping of the register
    # size - number of bytes to write or number of bytes to read
    def __init__(self, RnW, address, size, callback = None):
        self.RnW = RnW              
        self.address = address
        self.size = size
        self.callback = callback

class Msp430Interface:
    registers = {'SONAR'            : Register(True,    1, 8),
                 'ENCODERS_SPEED'   : Register(True,    2, 5),
                 'ENCODERS_POSITION': Register(True,    3, 8),
                 'MOTORS'           : Register(False,   4, 9),
                 'SET_STATUS'       : Register(False,   5, 1),
                 'GET_STATUS'       : Register(True,    6, 1)}

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

        self.comms_proxy = rospy.ServiceProxy('comms', CommSrv)
        rospy.loginfo('comms service up')

        # Publishers

        # Timers
        rospy.Timer(rospy.Duration(1./25), self.read_values)        

        # Subscribers

    # Reads in all read only registers and desiminates data
    def read_values(self, event):
        for reg in filter(lambda x: x.RnW == True, Msp430Interface.registers.itervalues()):
            read = self.comms_proxy(CommSrvRequest(
                RnW =           True,
                destination =   CommSrvRequest.MSP430,
                addr =          reg.address,
                data =          [0x55] * reg.size))
            print reg.address, [format(ord(i), "#04x") for i in read.data]

if __name__ == '__main__':
    msp = Msp430Interface()
    rospy.spin()