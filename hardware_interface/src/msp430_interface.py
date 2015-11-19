#!/usr/bin/env python

import rospy
from rospy import ServiceException
from hardware_interface.srv import CommRepetitiveRead,  CommRepetitiveReadRequest,  CommRepetitiveReadResponse
from hardware_interface.srv import CommSingleRead,      CommSingleReadRequest,      CommSingleReadResponse
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

class Register:
    modes = {'REPETITIVE_READ'  : 0,
             'SINGLE_READ'      : 1,
             'SINGLE_WRITE'     : 2}
    # mode - 0 for repetative read, 1 for single read, 2 for single write
    # address - mapping of the register
    # size - number of bytes to write or number of bytes to read
    # callback - on read this function is called to handle the data
    # pack - on write this funciton packs the data
    def __init__(self, mode, device, addr, size, pack = None, topic = None, callback = None):
        self.mode = mode
        self.device = device              
        self.addr = addr
        self.size = size
        self.callback = callback
        self.pack = pack
        self.topic = topic

        self.write_pub = rospy.Publisher('comm_single_write', CommSingleWrite, queue_size = 1)
        self.read_proxy = rospy.ServiceProxy('comm_single_read', CommSingleRead)

        # Set up a repetitive read
        if mode == Register.modes['REPETITIVE_READ']:
            if self.callback and self.topic:
                # Set up subscriber callback
                self.sub = rospy.Subscriber(self.topic, CommReadData, callback, queue_size = 1)

            # Register a repetitive read
            resp = rospy.ServiceProxy('comm_repetitive_read', CommRepetitiveRead)(
                CommRepetitiveReadRequest(
                    add         = True,
                    device      = self.device,
                    addr        = self.addr,
                    size        = self.size,
                    topic       = self.topic))

            if not resp.success:
                if resp.failure_code == CommRepetitiveReadResponse.ID_ALREADY_EXIST:
                    print 'log_warn'
                    rospy.logwarn('Repetitive read for device: %d, address: %d, size: %d, already exist' % (
                        self.device, self.addr, self.size))
                else:
                    rospy.logwarn('Repetitive read for device: %d, address: %d, size: %d, failed to add because error number %d' % (
                        self.device, self.addr, self.size, resp.failure_code))
    # Perform a single read
    def read(self):
        # Read the register
        read = self.read_proxy(CommSingleReadRequest(
                device =        self.device,
                addr =          self.addr,
                size =          self.size))

        # Perform callback if nesisary
        if self.callback is not None:
            self.callback(read)

    def write(self, *data):
        # Check conditions
        if self.pack is None:
            rospy.logerr('Write to Device: %d, Address: %d, Size: %d, does not have a pack method' % (
                self.device, self.addr, self.size))
            return

        # pack and send the data
        packed_data = self.pack(*data)
        self.write_pub.publish(CommSingleWrite(
                device =        self.device,
                addr =          self.addr,
                data =          packed_data))

    def stop_repetitive_read(self):
        if self.mode != Register.modes['REPETITIVE_READ']:
            return
        # Register a repetitive read
        resp = rospy.ServiceProxy('comm_repetitive_read', CommRepetitiveRead)(
            CommRepetitiveReadRequest(
                add         = False,
                device      = self.device,
                addr        = self.addr,
                size        = self.size,
                topic       = self.topic))

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
         'SONAR'            : Register(Register.modes['REPETITIVE_READ'], CommSingleReadRequest.MSP430, 1,  8,  callback = read_sonar,     topic = 'raw_sonar'),
         'ENCODERS_SPEED'   : Register(Register.modes['REPETITIVE_READ'], CommSingleReadRequest.MSP430, 2,  5,  callback = read_encoder,   topic = 'raw_encoder_speed'),
         'ENCODERS_POSITION': Register(Register.modes['REPETITIVE_READ'], CommSingleReadRequest.MSP430, 3,  8,  callback = None),
         'MOTORS'           : Register(Register.modes['SINGLE_WRITE'],    CommSingleReadRequest.MSP430, 4,  5,  pack =     pack_pwm),
         'SET_STATUS'       : Register(Register.modes['SINGLE_WRITE'],    CommSingleReadRequest.MSP430, 5,  1),
         'GET_STATUS'       : Register(Register.modes['REPETITIVE_READ'], CommSingleReadRequest.MSP430, 6,  1,  callback = None,           topic = 'raw_status')}


        # Timers
        #rospy.Timer(rospy.Duration(1./25), self.read_values)        

        # Subscribers
        self.pwm_sub = rospy.Subscriber("motor_pwm", MotorPWM, self.pwm_cb, queue_size = 1)
    
    # Reads in all read only registers
    def read_values(self, event):
        for reg in filter(lambda x: x.mode == Register.modes['SINGLE_READ'], Msp430Interface.registers.itervalues()):
            reg.read()

    def stop_repetitive_reads(self):
        for reg in filter(lambda x: x.mode == Register.modes['REPETITIVE_READ'], Msp430Interface.registers.itervalues()):
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
