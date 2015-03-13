#!/usr/bin/python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# Copyright (c) 2015, Fetch Robotics, Inc.
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
##\brief Reads voltage and current measurements from DSO scope to determine power
##        disapation of hotswap FET. 

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

from __future__ import print_function
import sys
import getopt
import re
import pylab
import time

from scpi_lxi import LXIDevice
from dso6054a import DSO6054A

def usage(progname):
    print(__doc__ % vars())

def main():
    progname = sys.argv[0]
    optlist,argv = getopt.gnu_getopt(sys.argv[1:], "h");    

    dev = None
    for opt,arg in optlist:
        if (opt == "-h"):
            usage(progname)
            return 0
        else :
            print("Internal error : opt = ", opt)
            return 2

    if len(argv) != 2:
        usage(progname)
        return 1

    address = argv[0]
    supply_voltage = float(argv[1])

    print("Connecting to LXI device using network address %s" % address)
    dev = LXIDevice(address)
    scope = DSO6054A(dev)

    samples = 1000
    xinc,voltage = scope.read_waveform(1, samples)
    xinc,current = scope.read_waveform(4, samples)

    print('supply voltage = ', supply_voltage)
    mosfet_voltage = supply_voltage- pylab.array(voltage) 
    power = current * mosfet_voltage
    
    t = pylab.arange(len(voltage)) * xinc
    t_ms = t * 1e3

    charge = [0.0]
    for c in current:
        charge.append(charge[-1]+c*xinc)
    charge = pylab.array(charge[1:])
    # find first point where voltage ~= supply voltage
    voltage_up_index = None
    for i,v in enumerate(voltage):
        if supply_voltage - v < 0.1:
            voltage_up_index = i
            break
    capacitance = charge[i] / supply_voltage
    if voltage_up_index is not None:
        capacitance = charge[i] / supply_voltage
        print("Estimated capacitance %0.1fuF" % (capacitance * 1e6))
    else:
        capacitance = charge[i] / supply_voltage
        print("Cannot accurrately estimate capacitance because voltage did not come up") 
        print("Minimum estimated capacitance %0.1fuF" % (capacitance * 1e6))

    # Estimate turn-on time by looking at power waveform
    power_mid = 0.5 * (max(power) + min(power))
    power_high = list(power > power_mid)  # true if power if higher than mid point
    rising = power_high.index(True)
    try:
        # find falling edge, but start at least 10uSec after rising edge
        falling = power_high.index(False,rising+max(1, int(10e-6/xinc)))
    except ValueError:
        print("Couldn't find falling edge, assuming end for trace")
        falling = len(power)-1
    rising,falling = (t_ms[rising],t_ms[falling])
    print("Turn-on time %.2f ms" % (falling-rising))
    
    # estimate power limit by looking at waveform    
    power_limit = pylab.extract( (power>(0.95*max(power))) , power).mean()
    print("Power limit seems to be %.1f Watts" % power_limit)

    
    pylab.figure()
    pylab.subplot(3,1,1)
    pylab.plot(t_ms,current)
    pylab.xlabel('time (ms)')
    pylab.ylabel('current (A)')
    pylab.subplot(3,1,2)
    pylab.plot(t_ms,mosfet_voltage, label='mosfet voltage drop')
    pylab.plot(t_ms,voltage, label='supply voltage')
    pylab.xlabel('time (ms)')
    pylab.ylabel('voltage (V)')
    pylab.legend()
    pylab.subplot(3,1,3)
    pylab.plot(t_ms,power,'r')
    pylab.plot([t_ms[0],t_ms[-1]],[power_mid,power_mid],'k--')
    pylab.plot([rising,rising],[min(power),max(power)],'k--')
    pylab.plot([falling,falling],[min(power),max(power)],'k--')
    pylab.plot([t_ms[0],t_ms[-1]],[power_limit, power_limit],'b-.')
    pylab.xlabel('time (ms)')
    pylab.ylabel('power (Watt)')    

    pylab.figure()
    pylab.xlabel('time (ms)')
    pylab.ylabel('charge (Columbs)')
    pylab.plot(t_ms,charge)
    if voltage_up_index is not None:
        pylab.plot(t_ms[voltage_up_index],charge[voltage_up_index],'r*')
      
    pylab.show()
    
    
    


if __name__ == '__main__':
    main()
