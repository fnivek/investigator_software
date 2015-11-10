#!/usr/bin/env python

import rospy
from hardware_interface.srv import CommSrv, CommSrvRequest

class ImuInterface:
    # Dictionary of register names and adresses
    registers = {'FUNC_CFG_ACCESS': 0x01,
                 'WHO_AM_I'     :   0x0F,
                 'CTRL1_XL'     :   0x10,
                 'CTRL2_G'      :   0x11,
                 'CTRL3_C'      :   0x12,
                 'CTRL4_C'      :   0x13,
                 'CTRL8_XL'     :   0x17,
                 'CTRL10_C'     :   0x19,
                 'OUTX_L_G'     :   0x22,
                 'OUTX_H_G'     :   0x23,
                 'OUTY_L_G'     :   0x24,
                 'OUTY_H_G'     :   0x25,
                 'OUTZ_L_G'     :   0x26,
                 'OUTZ_H_G'     :   0x27,
                 'OUTX_L_XL'    :   0x28,
                 'OUTX_H_XL'    :   0x29,
                 'OUTY_L_XL'    :   0x2A,
                 'OUTY_H_XL'    :   0x2B,
                 'OUTZ_L_XL'    :   0x2C,
                 'OUTZ_H_XL'    :   0x2D
                }

           
    def __init__(self):
        rospy.init_node('imu_interface', anonymous=False)

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

        # Disable I2C
        """
        self.comms_proxy(CommSrvRequest(
            RnW =           False,
            destination =   CommSrvRequest.IMU,
            addr = ImuInterface.registers['CTRL4_C'],
            data =          [0x04]))
        """
        
        """
        for i in xrange(0x1, 0x6C):
            read = self.comms_proxy(CommSrvRequest(
                RnW =           True,
                destination =   CommSrvRequest.IMU,
                addr =          i,
                data =          [0x55]))
            print format(i, '#04x'), '\t\t:\t', format(ord(read.data), '#04x'), ord(read.data)

        """
        while not rospy.is_shutdown():
            read = self.comms_proxy(CommSrvRequest(
                RnW =           True,
                destination =   CommSrvRequest.IMU,
                addr =          ImuInterface.registers['WHO_AM_I'],
                data =          [0x55]))
            print format(ord(read.data), '#04x'), '\t:\t',  ord(read.data)
        
if __name__ == '__main__':
    imu = ImuInterface()
