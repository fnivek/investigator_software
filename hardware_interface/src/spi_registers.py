#!/usr/bin/env python

import rospy
from hardware_interface.srv import CommRepetitiveRead,  CommRepetitiveReadRequest,  CommRepetitiveReadResponse
from hardware_interface.srv import CommSingleRead,      CommSingleReadRequest,      CommSingleReadResponse
from hardware_interface.msg import CommSingleWrite,     CommReadData

class SPIRegister:
    modes = {'REPETITIVE_READ'  : 0,
             'SINGLE_READ'      : 1,
             'SINGLE_WRITE'     : 2}
    devices = {'MSP430'         : CommSingleReadRequest.MSP430,
               'IMU'            : CommSingleReadRequest.IMU}
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
        if mode == SPIRegister.modes['REPETITIVE_READ']:
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
        if self.mode != SPIRegister.modes['REPETITIVE_READ']:
            return
        # Register a repetitive read
        resp = rospy.ServiceProxy('comm_repetitive_read', CommRepetitiveRead)(
            CommRepetitiveReadRequest(
                add         = False,
                device      = self.device,
                addr        = self.addr,
                size        = self.size,
                topic       = self.topic))
