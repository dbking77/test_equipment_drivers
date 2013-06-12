#!/usr/bin/python
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
##\brief Interface Omega HH147 Rs-232 Data Logger Thermometer

"""
Interface Omega HH147 RS-232 Data Logger Thermometer.  
Reads 4x temperature values and publishes them to ROS
"temperature" topic.

Usage: %(progname)s [-h] <serial_dev>

Options:
  serial_dev : Serial device thermometer is connected to.
  -h : show this help

Example:
  %(progname)s /dev/ttyUSB0

To view data try using rxplot: 
 rxplot -b100 -p100 temperature/data[0]:data[1]:data[2]:data[3]

Serial Port Connector:

  
  3.5mm Audio Jack
 
       Gnd      TX        RX
        |        |         |
  |||   V        V         V
  |||-------,,------,,___,,,---,
  |||       ||      ||          \
  |||       ||      ||           |
  |||       ||      ||___       /
  |||-------''------''   '''---'
  |||

 Note : The TX pin is an output and RX is an input.
         When connecting the 3.5Audio Jack to computer, 
         connect the Audio Jack TX to the computer RX
         and connect the Audio Jack RX to the computer TX

"""

PKG = 'omega_hh147'
import roslib
roslib.load_manifest(PKG)

import rospy
import serial
import sys
import std_msgs.msg
import math

def usage(progname):
  print __doc__ % vars()


def msgPrint(msg):
    print "msg :"
    for val in msg:
        print " %02x" % val
    print

def bcd(val):
    if (val > 9) or (val < 0):
        raise RuntimeError("Invalid BCD value %X" % val)
    return val

def parseTemp(msg):
    if len(msg) != 4:
        raise RuntimeError("Invalid len : %d" % len(msg))

    flags = msg[2]>>4
    temp  = 1000. * bcd(msg[2]&0xF)
    temp += 100.  * bcd(msg[1]>>4)
    temp += 10.   * bcd(msg[1]&0xF)
    temp += 1.    * bcd(msg[0]>>4)
    temp += 0.1   * bcd(msg[0]&0xF)

    if flags & 0x1:
        temp = -temp

    if flags & 0x2:
        return float('nan')

    return temp


def parseMessage(msg):
    if len(msg)!=25:
        msgPrint(msg)
        raise RuntimeError("Invalid message length")

    if (msg[0] != 0xAA) or (msg[-1] != 0xAB):
        raise RuntimeError("Invalid message frame")

    if msg[1] != 0xB1:
        raise RuntimeError("Do not understand message type %02x" % msg[1])

    t1 = parseTemp(msg[8:12])
    t2 = parseTemp(msg[12:16])
    t3 = parseTemp(msg[16:20])
    t4 = parseTemp(msg[20:24])

    return (t1,t2,t3,t4)


def main():
    
    progname = sys.argv[0]
    import getopt
    optlist,argv = getopt.gnu_getopt(sys.argv[1:], "h");

    dev = None
    for opt,arg in optlist:
        if (opt == "-h"):
            usage(progname)
            return 0
        else :
            print "Internal error : opt = ", opt
            return 2

    if len(argv) != 1:
      usage(progname)
      return 1

    device_name = argv[0]
    dev = serial.Serial(device_name, 9600, timeout=0.2, parity=serial.PARITY_NONE, rtscts=0)

    rospy.init_node('omega_hh147', anonymous=True)

    pub = rospy.Publisher("temperature", std_msgs.msg.Float64MultiArray)
    dim0 = std_msgs.msg.MultiArrayDimension(label="temperature", size=4, stride=4)
    layout = std_msgs.msg.MultiArrayLayout(dim=[dim0], data_offset=0)

    buf = []
    last_msg_time = rospy.Time.now()
    last_err_time = rospy.Time.now()
    while True:
        tmp_str = dev.read(25)
        # convert string into list of integers
        buf += [ ord(char) for char in tmp_str ]
        
        # slice buffer up into discrete messages by lookign for continuous chunks starting with 0xAA and ending with 0xAB
        while (buf.count(0xAA) > 0) and (buf.count(0xAB) > 0):
            start_index = buf.index(0xAA)
            stop_index = buf.index(0xAB)
            if start_index > 0:
                print "Warning, throwing away %d elements" % start_index
                buf = buf[start_index:]
            else:
                msg = buf[start_index:stop_index+1]
                buf = buf[stop_index+1:]
                temp_list = parseMessage(msg)
                pub.publish(layout=layout, data=temp_list)
                last_msg_time = rospy.Time.now()

        now = rospy.Time.now() 
        if (now - last_msg_time).to_sec() > 5.0:
            if (now - last_err_time).to_sec() > 5.0:
                last_err_time = now
                print "Haven't received new in %f seconds" % math.floor((now - last_msg_time).to_sec() + 0.5)

        if len(buf) > 0:
            print "%d bytes left over" % len(buf)
                        


if __name__ == '__main__':
    main()


