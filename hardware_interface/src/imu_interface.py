#!/usr/bin/env python

import rospy
from hardware_interface.srv import CommSingleRead,      CommSingleReadRequest,      CommSingleReadResponse

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


        # Comms service proxy
        # Wait for service to come up
        rospy.loginfo('Waiting for comms...')
        for service in ['comm_single_read', 'comm_repetitive_read']:
            while not rospy.is_shutdown():
                try:
                    rospy.wait_for_service(service, 1)
                    break
                except Exception as e:
                    rospy.logerr('Timeout waiting for %s service' % service)


        self.single_read_proxy = rospy.ServiceProxy('comm_single_read', CommSingleRead)
        rospy.loginfo('comms service up')

        # Disable I2C
        """
        self.single_read_proxy(CommSingleReadRequest(
            RnW =           False,
            device =   CommSingleReadRequest.IMU,
            addr = ImuInterface.registers['CTRL4_C'],
            size =          [0x04]))
        """
        
        """
        for i in xrange(0x1, 0x6C):
            read = self.single_read_proxy(CommSingleReadRequest(
                RnW =           True,
                device =   CommSingleReadRequest.IMU,
                addr =          i,
                size =          [0x55]))
            print format(i, '#04x'), '\t\t:\t', format(ord(read.data), '#04x'), ord(read.data)

        """
        while not rospy.is_shutdown():
            read = self.single_read_proxy(CommSingleReadRequest(
                device =   CommSingleReadRequest.IMU,
                addr =     ImuInterface.registers['WHO_AM_I'],
                size =     1))
            print format(ord(read.data), '#04x'), '\t:\t',  ord(read.data)
        
if __name__ == '__main__':
    imu = ImuInterface()
