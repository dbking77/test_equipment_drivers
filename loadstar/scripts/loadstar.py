#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#


##\author Derek King
##\brief Driver to Loadstar DQ-1000U frequency to USB interface

"""
Driver to Loadstar DQ-1000U frequency to USB interface
Reads force value and publishes them to ROS "load" topic. 
The force values are published in Newtons, although they 
are read from device in millipounds.

The DQ1000U uses a FDTI USB-to-RS232 converter internally, 
so it shows up as a serial device.  
In Linux, the device usually named /dev/ttyUSB?? where ??
is a number starting at zero.  It is possible to use 
dmesg after plugging device in to determine what name 
device was assigned.

Also, you may need to change write/read permissions on 
Linux device file, so non-root program can communicate
with sensor. 

Usage: %(progname)s [-h] <serial_dev>

Options:
  serial_dev : serial device used to talk to device.  
  -a <num>   : take average of <num> samples before publishing.
  -z         : zero load data on start up
  -r <rate>  : change program sample rate (in Hz).  
               Note, at sample rate may top out about 50Hhz depending on USB latency.
  -h : show this help

Example 1:
  %(progname)s /dev/ttyUSB0

Exmample 2:  
  %(progname)s /dev/ttyUSB0 -a 5 -z -r20 

To view data try using rxplot: 
 rxplot -b100 -p100 load
"""

PKG = 'loadstar'
import roslib
roslib.load_manifest(PKG)

import rospy
import serial
import sys
import std_msgs.msg

def usage(progname):
  print __doc__ % vars()


def printHex(msg):
    buf = [ ord(char) for char in msg ]
    print buf


class Loadstar:
    def __init__(self, device_name):
        self.dev = serial.Serial(device_name, 230400, timeout=0.5, parity=serial.PARITY_NONE, rtscts=0)

        # first write '\n' to stop any ongoing transfer, then flush buffer
        self.write('')
        for i in range(5):
            response = self.dev.read(100)
            if len(response) == 0:
                break
            #print "Flushed %d bytes : '%s'" % (len(response), response)

        if len(response) != 0:        
            raise RuntimeError("Device won't stop sending data.  Last repsonse = %s" % response)

        # try to verify that serial device is actually loadstar sensor, first send 
        # newline and wait for device to respond with "A"
        self.write('')
        response = self.dev.read(3)
        if response[-3:] != 'A\r\n':
            printHex(response)
            raise RuntimeError("Unexpected device initialization response : '%s' (%d) expected 'A'" % (response, len(response)))
        self.write('SS0')
        self.model_number = self.readline()
        
        self.write('SS1')
        self.serial_number = self.readline()

        LBS_TO_NEWTONS = 4.448222
        self.write('SLC')
        response = self.readline()
        self.load_capacity = LBS_TO_NEWTONS * float(response)

    def getLoad(self):
        self.write('O0W1')
        response = self.readline()
        LBS_TO_NEWTONS = 4.448222
        load_newtons = LBS_TO_NEWTONS * int(response) * 0.001
        return load_newtons
        
    def printDeviceInfo(self):
        print "Model Number     : %s" % self.model_number
        print "Serial Number    : %s" % self.serial_number
        print "Load Capacity    : %f Newtons" % self.load_capacity
        print "Load Measurement : %f Newtons" % self.getLoad()

    def write(self,cmd):
        self.dev.write(cmd + '\r\n')

    def readline(self):
        response = self.dev.readline()
        if response[-2:] != '\r\n':
            raise RuntimeError("Reponse from device d/n end with cariage return and newline")
        return response[:-2]


def main():
    
    progname = sys.argv[0]
    import getopt
    optlist,argv = getopt.gnu_getopt(sys.argv[1:], "hza:r:");

    dev = None
    num_samples = 1
    use_zero = False
    rate_hz = 20
    for opt,arg in optlist:
        if (opt == "-h"):
            usage(progname)
            return 0
        elif (opt == "-z"):
            use_zero = True
        elif opt == '-a':
            num_samples = int(arg)
        elif opt == '-r':
            rate_hz = float(arg)
        else :
            print "Internal error : opt = ", opt
            return 2

    if len(argv) != 1:
      usage(progname)
      return 1

    device_name = argv[0]
    loadstar = Loadstar(device_name)
    loadstar.printDeviceInfo()


    rospy.init_node('loadstar', anonymous=True)

    pub = rospy.Publisher("load", std_msgs.msg.Float64)

    if use_zero:
        print
        print "Taking average of 100 samples to zero sensor with..."
        samples = []
        while len(samples) < 100:
            samples.append(loadstar.getLoad())
        zero = float(sum(samples)) / float(len(samples))        
        print "Using zero offset of : %f" % zero
    else:
        zero = 0.0

    print
    print "To view data use : "
    print "  rxplot -b100 -p100 load/data"
    samples = []
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        samples.append(loadstar.getLoad())
        #print load
        if len(samples) >= num_samples:
            avg = float(sum(samples)) / float(len(samples))
            pub.publish(std_msgs.msg.Float64(avg-zero))
            samples = []
        rate.sleep()


if __name__ == '__main__':
    main()



