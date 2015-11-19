#!/usr/bin/env python
import rospy
import serial
import spidev
import gpio
import time
import threading
from collections import deque
from struct import *
from hardware_interface.msg import CommSingleWrite,     CommReadData
from hardware_interface.srv import CommSingleRead,      CommSingleReadRequest,      CommSingleReadResponse
from hardware_interface.srv import CommRepetitiveRead,  CommRepetitiveReadRequest,  CommRepetitiveReadResponse

class Single:
    def __init__(self, RnW, device, addr, size, data = None, topic = None):
        self.RnW = RnW
        self.device = device
        self.addr = addr 
        self.size = size
        self.data = data
        self.pub = None
        if topic is not None and topic != '':
            self.pub = rospy.Publisher(topic, CommReadData, queue_size = 1)

        self.gen_uid()

    # Generate a unique id given RnW, device, addr, and size
    #   To keep it simple id is defined as follows 
    #       bits 0-7 are addr
    #       bit 8 is 1 if read 0 if write
    #       bits 9-11 are the device
    #       bits 12-19 are the size
    def gen_uid(self):
        self.uid = (self.size << 12) | (self.device << 9) | (self.RnW << 8) | self.addr

    def get_write(self):
        if self.RnW:
            return [self.addr] + [0x55 for i in xrange(self.size)]
        else:
            return [self.addr] + [ord(self.data[i]) for i in xrange(self.size)]

class Comms:
    def __init__(self):
        #Init ros
        rospy.init_node('comms', anonymous=False)

        # Grab parameters
        # If emulate_odroid the program will use UART to communicate with an MSP430 set up to 
        #   convert UART to SPI (see https://github.com/fnivek/Investigator_UartToSPI)
        self.emulate_odroid = rospy.get_param('~emulate_odroid', False)
        # Port is the serial port name to connect to which will be different based on emulate odroid 
        self.port = rospy.get_param('~port', '/dev/serial/by-id/usb-Texas_Instruments_Texas_Instruments_MSP-FET430UIF_B5FF4A8DEDD03815-if00')
        
        # Get gpio
        temp_gpio = {CommSingleReadRequest.IMU : 21,
                     CommSingleReadRequest.MSP430 : 18}
        self.cs = {CommSingleReadRequest.IMU : None,
                     CommSingleReadRequest.MSP430 : None}
        for device, pin in temp_gpio.iteritems():
            try:
                self.cs[device] = gpio.GPIO(pin)
                self.cs[device].direction = 'out'
            except IOError as err:
                if err.errno == 13:
                    # This is an expected error, the udev rule is slower than python
                    time.sleep(0.1)
                    self.cs[device] = gpio.GPIO(pin)
                    self.cs[device].direction = 'out'
                else:
                    # This is an unexpected IOError
                    rospy.logerr('Unexpected IOError when accessing gpio')
                    raise

        # Attempt to connect to the serial port
        rospy.loginfo('Attempting to connect to serial interface')
        while not rospy.is_shutdown():
            try:
                if self.emulate_odroid:
                    self.iface = serial.Serial(self.port, 9600, timeout=1)
                    rospy.loginfo('Connected to %s' % self.port)
                else:
                    self.iface = spidev.SpiDev()
                    self.iface.open(1, 0)
                    # Mode 0b01 works with MSP in mode 00
                    self.iface.mode = 0b01	#CPHA = 0, CPPOL = 1
                    self.iface.max_speed_hz = 5000
                    rospy.loginfo('Connected to spi')
                break;
            except Exception as e:
                rospy.logerr('Failed to connect to serial interface\n\tException: %s' % str(e))
                time.sleep(1)

        # Lock for muatating repetitive_reads deque
        self.lock = threading.Lock()

        # Using deques because they are thread safe
        #   TODO: Use more efficent types that are still thread safe
        self.singles_deque = deque([])          # This one should be a deque
        self.completed_single_reads = deque([]) # This should be some map type such as a dictionary
        self.repetitive_reads = deque([])       # This should be a type that is quick to itterate over

        # Initilize Subscribers
        self.single_write_sub = rospy.Subscriber('comm_single_write', CommSingleWrite, self.single_write_cb, queue_size = 5)

        # Initilize services
        self.single_read_srv = rospy.Service('comm_single_read', CommSingleRead, self.single_read_cb)
        self.repetitive_read_srv = rospy.Service('comm_repetitive_read', CommRepetitiveRead, self.repetitive_read_cb)

    def spi_read_write(self, single):
        write = single.get_write()
        read = []
        self.cs[single.device].value = 0
        for byte in write:
            read += self.iface.xfer([byte])
            time.sleep(0.000001)
        self.cs[single.device].value = 1
        return read[1:]


    # This function handles all of the repetitive reads and queued single read and writes
    #   as fast as possible
    def run(self):
        while not rospy.is_shutdown():
            # Handle repetitive reads
            with self.lock:
                for read in self.repetitive_reads:
                    data = self.spi_read_write(read)
                    if read.pub:
                        read.pub.publish(data)

            # Itterate through all single read and writes
            while len(self.singles_deque):
                single = self.singles_deque.popleft()
                single.data = self.spi_read_write(single)
                if single.RnW:
                    self.completed_single_reads.append((single.uid, single.data))


    def single_read_cb(self, req):
        single = Single(True, req.device, req.addr, req.size)
        uid = single.uid
        self.singles_deque.append(single)

        # Wait until the run function has processed the read or 5 seconds have passed
        now = rospy.get_time()
        while rospy.get_time() - now < 5.0:
            for item in self.completed_single_reads:
                if item[0] == uid:
                    return CommSingleReadResponse(item[1])

        return None

    def single_write_cb(self, msg):
        single = Single(False, msg.device, msg.addr, len(msg.data), data = msg.data)
        self.singles_deque.append(single)

    def repetitive_read_cb(self, req):
        single = Single(True, req.device, req.addr, req.size, topic = req.topic)
        # Check if it already exist
        existing_single = None
        for s in self.repetitive_reads:
            if s.uid == single.uid:
                existing_single = s

        if existing_single and req.add:
            return CommRepetitiveReadResponse(
                success = False,
                failure_code = CommRepetitiveReadResponse.ID_ALREADY_EXIST)
        elif not req.add and not existing_single:
            return CommRepetitiveReadResponse(
                success = False,
                failure_code = CommRepetitiveReadResponse.NO_ID_MATCH)
        else:
            if req.add:
                with self.lock:
                    self.repetitive_reads.append(single)
            else:
                with self.lock:
                    self.repetitive_reads.remove(existing_single)

            return CommRepetitiveReadResponse(
                success = True,
                failure_code = CommRepetitiveReadResponse.OK)


    """ Communication service callback
            Attempts to write the data 
    """
    def comm_cb(self, req):
        # Set the RnW bit in the address
        addr = req.addr & 0x7F
        addr = addr | (req.RnW << 7)
        
        size = len(req.data)
        read = []
        write = [addr] + [ord(req.data[i]) for i in xrange(size)]
        with self.lock:
            self.cs[req.destination].value = 0
            
            for byte in write:
                read += self.iface.xfer([byte])
                time.sleep(0.000001)
            self.cs[req.destination].value = 1

        # Attempt to read if RnW
        resp = CommSingleReadResponse()
        if req.RnW:
            try:
                resp.data = read[1:]
            except Exception as e:
                rospy.logerr('Failed to read ' + str(size) + 'bytes from ' + self.port + 
                    '.\n\tException: ' + str(e))
                return None

        return resp

    def __del__(self):
        if self.iface != None:
            self.iface.close()


if __name__ == '__main__':
    c = Comms()
    c.run()
    del(c)
