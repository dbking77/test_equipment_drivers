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
##\brief Interface to Prologix GPIB-Ethernet controller

"""
Grabs current and voltage data from oscilliscope, and plots power over hotswap mosfet

Usage: %(progname)s [-h] <address> <voltage>

Options:
  address : Use address to connect to LXI device. Address can be IPv4 address or hostname.
  voltage : supply voltage (used to calculate voltage drop over hotswap MOSFET)
  -h : show this help

Probe Setup:
 Ch1 : hotswap mosftet output voltage
 Ch4 : hotswap output current

Example:
  %(progname)s 10.0.1.197 24.0

"""

PKG = 'dso6054a'

import roslib; roslib.load_manifest(PKG)
import rospy

from scpi_lxi.scpi_lxi import LXIDevice
from dso6054a.dso6054a import DSO6054A

import re
import sys

import pylab

def usage(progname):
  print __doc__ % vars()

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

    if len(argv) != 2:
      usage(progname)
      return 1

    address = argv[0]
    supply_voltage = float(argv[1])

    print "Connecting to LXI device using network address %s" % address
    dev = LXIDevice(address)
    scope = DSO6054A(dev)    
    
    #def.write("*RST")


    if False:

        w = dev.write
        r = dev.read

        w("*IDN?")
        idn = r()
        print idn
        #'AGILENT TECHNOLOGIES,DSO6054A,MY44008014,04.10.0239'
        if not re.match('AGILENT TECHNOLOGIES,DSO6054A',idn):
            print "Bad indentification : ", idn

        #w(":digitize channel2")

        w(":waveform:format ascii")
        w(":waveform:format?")
        print 'format:', r()

        w(":waveform:source channel2")
        w(":waveform:source?")
        print 'source:', r()

        w(":waveform:points:mode normal")
        w(":waveform:points 100")
        w(":waveform:points?")
        print 'points:', r()   

        w(":waveform:points:mode?")
        print 'pointsmode:', r()

        pre = r('waveform:preamble?')
        format,typ,points,count,xinc,xorg,xref,yinc,yorg,yref = pre.split(',')
        print "preamble", 

        xinc = float(xinc)

        print 'format', format
        print 'type', typ
        print 'points', points
        print 'count', count
        print 'xinc', xinc
        print 'xorg', xorg
        print 'xref', xref
        print 'yinc', yinc
        print 'yorg', yorg
        print 'yref', yref


        print r()
        data = r(':waveform:data?')
        hdr = data[0:2]
        if hdr != "#8":
            print "Warning, Unexpected data header", hdr
        size = int(data[2:10])
        data = data[10:]
        if size != len(data)-1:
            print "Warning : data length mismatch"

        # break data into array of floats
        data = [float(d) for d in data.split(',')]
        #print len(data)
        #print data

    samples = 1000
    xinc,voltage = scope.read_waveform(1, samples)
    xinc,current = scope.read_waveform(4, samples)

    print 'supply voltage = ', supply_voltage
    mosfet_voltage = supply_voltage- pylab.array(voltage) 
    power = current * mosfet_voltage
    
    t = pylab.arange(len(voltage)) * xinc * 1e3
    
    pylab.figure()
    pylab.subplot(3,1,1)
    pylab.plot(t,current)
    pylab.xlabel('time (ms)')
    pylab.ylabel('current (A)')
    pylab.subplot(3,1,2)
    pylab.plot(t,mosfet_voltage, label='mosfet voltage drop')
    pylab.plot(t,voltage, label='supply voltage')
    pylab.xlabel('time (ms)')
    pylab.ylabel('voltage (V)')
    pylab.legend()
    pylab.subplot(3,1,3)
    pylab.plot(t,power)
    pylab.xlabel('time (ms)')
    pylab.ylabel('power (Watt)')
    pylab.show()
    

    # print 'x-inc', r(':waveform:xincrement?')
    # print 'y-inc', r(':waveform:yincrement?')
    
    # print 'y-origin', r(':waveform:yorigin?')
    # print 'y-ref', r(':waveform:yreference?')
    
    
    


if __name__ == '__main__':
    main()
