#!/usr/bin/env python
import rospy
import serial
import spidev
import gpio
import time
import threading
from struct import *
from hardware_interface.srv import CommSrv, CommSrvRequest, CommSrvResponse

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
        self.gpio = {CommSrvRequest.IMU : 21,
                     CommSrvRequest.MSP430 : 18}
        self.cs = {CommSrvRequest.IMU : None,
                     CommSrvRequest.MSP430 : None}
        for device, pin in self.gpio.iteritems():
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
                    self.iface.max_speed_hz = 10000
                    rospy.loginfo('Connected to spi')
                break;
            except Exception as e:
                rospy.logerr('Failed to connect to serial interface\n\tException: %s' % str(e))
                time.sleep(1)

        # Threading lock so only one entity uses the iface
        self.lock = threading.Lock()

        # Initilize service
        self.service = rospy.Service('comms', CommSrv, self.comm_cb)

        # Wait for shutdown
        self.service.spin()

    """ Communication service callback
            Attempts to write the data 
    """
    def comm_cb(self, req):
        # Set the RnW bit in the address
        addr = req.addr & 0x7F
        addr = addr | (req.RnW << 7)
        
        size = len(req.data)
        read = ''
        write = [addr] + [ord(req.data[i]) for i in xrange(size)]
        with self.lock:
            self.cs[req.destination].value = 0
            if self.emulate_odroid:
                # Pack the data and write data
                data = pack('<B' + 'B' * size, *write)
                self.iface.flushInput()     # Ensure there is no pending data
                self.iface.write(data)
            else:
                read = self.iface.xfer(write)
            self.cs[req.destination].value = 1

        # TODO: Select correct destination CS

        # Attempt to read if RnW
        resp = CommSrvResponse()
        if req.RnW:
            try:
                if self.emulate_odroid:
                    with self.lock:
                        resp.data = unpack('<' + 'B' * size, self.iface.read(size))
                else:
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
    del(c)