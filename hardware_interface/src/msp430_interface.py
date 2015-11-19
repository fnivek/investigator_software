#!/usr/bin/env python

import rospy
from spi_registers import SPIRegister
from rospy import ServiceException
from hardware_interface.msg import MotorPWM, EncoderSpeed, CommSingleWrite, CommReadData
from struct import pack, unpack

encoder_speed_pub = rospy.Publisher('encoder_speed', EncoderSpeed, queue_size = 1)

# pack up pwm signals
def pack_pwm(left_pwm, right_pwm):
    # Forward = 0
    # Reverse = 1
    l_dir = 0                       
    if left_pwm < 0: l_dir = 1      
    r_dir = 0
    if right_pwm < 0: r_dir = 1

    # Set up direction for packing
    direction = l_dir | (r_dir << 1)

    return pack('<BHH', direction, abs(left_pwm), abs(right_pwm))

# Print whatever was read in hex
def print_hex_cb(resp):
    print [format(ord(i), "#04x") for i in resp.data]

# Callback to send out data about encoder speed
def read_encoder(resp):
    (direction, left_speed, right_speed) = unpack('<BHH', resp.data)
    if(direction & 0x1):
        left_speed = -left_speed
    if(direction & 0x2):
        right_speed = -right_speed

    encoder_speed_pub.publish(EncoderSpeed(left_motor = left_speed, right_motor = right_speed))

def read_sonar(resp):
    sonar = unpack('<HHHH', resp.data)

class Msp430Interface:
    def __init__(self):
        rospy.init_node('msp430_interface', anonymous=False)

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

        
        rospy.loginfo('comms service up')

        Msp430Interface.registers = registers = {
         'SONAR'            : SPIRegister(SPIRegister.modes['REPETITIVE_READ'], SPIRegister.devices['MSP430'], 1,  8,  callback = read_sonar,     topic = 'raw_sonar'),
         'ENCODERS_SPEED'   : SPIRegister(SPIRegister.modes['REPETITIVE_READ'], SPIRegister.devices['MSP430'], 2,  5,  callback = read_encoder,   topic = 'raw_encoder_speed'),
         'ENCODERS_POSITION': SPIRegister(SPIRegister.modes['REPETITIVE_READ'], SPIRegister.devices['MSP430'], 3,  8,  callback = None),
         'MOTORS'           : SPIRegister(SPIRegister.modes['SINGLE_WRITE'],    SPIRegister.devices['MSP430'], 4,  5,  pack =     pack_pwm),
         'SET_STATUS'       : SPIRegister(SPIRegister.modes['SINGLE_WRITE'],    SPIRegister.devices['MSP430'], 5,  1),
         'GET_STATUS'       : SPIRegister(SPIRegister.modes['REPETITIVE_READ'], SPIRegister.devices['MSP430'], 6,  1,  callback = None,           topic = 'raw_status')}


        # Timers
        #rospy.Timer(rospy.Duration(1./25), self.read_values)        

        # Subscribers
        self.pwm_sub = rospy.Subscriber("motor_pwm", MotorPWM, self.pwm_cb, queue_size = 1)
    
    # Reads in all read only registers
    def read_values(self, event):
        for reg in filter(lambda x: x.mode == SPIRegister.modes['SINGLE_READ'], Msp430Interface.registers.itervalues()):
            reg.read()

    def stop_repetitive_reads(self):
        for reg in filter(lambda x: x.mode == SPIRegister.modes['REPETITIVE_READ'], Msp430Interface.registers.itervalues()):
            try:
                reg.stop_repetitive_read()
            except ServiceException:
                # ros will be shutdown so we won't get a response but thats okay
                #   The request still gets serviced
                pass

    def pwm_cb(self, pwm):
        Msp430Interface.registers['MOTORS'].write(pwm.left_pwm, pwm.right_pwm)

if __name__ == '__main__':
    msp = Msp430Interface()
    rospy.spin()
    msp.stop_repetitive_reads()
