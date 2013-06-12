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
Use current and voltage data from oscilliscope to measure inductance 

Usage: %(progname)s [-h] <address> <resistance> <settling time> <edge time>

Options:
  address       : Use address to connect to LXI device. Address can be IPv4 address or hostname.
  resistance    : Terminal resitance of motor winding
  settling time : Time after switching edge that is takes for current to settle
  -h : show this help

Example:
  %(progname)s osc2 2.1 0.5-e6 0.1e-6

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

    if len(argv) != 4:
      usage(progname)
      return 1

    address = argv[0]
    resistance = float(argv[1])
    settling_time = float(argv[2])
    edge_time = float(argv[3])

    print "Connecting to LXI device using network address %s" % address
    dev = LXIDevice(address)
    scope = DSO6054A(dev)    
    

    samples = 1000
    xinc,voltage = scope.read_waveform(1, samples)
    xinc,current = scope.read_waveform(4, samples)

    voltage = pylab.array(voltage)
    current = pylab.array(current)

    # find rising edge of voltage data
    edge_voltage = voltage.mean() * 0.5
    output_state = (voltage > edge_voltage)

    rising_edges  =  output_state[1:] & ~output_state[:-1]
    falling_edges = ~output_state[1:] &  output_state[:-1]

    if not any(rising_edges) and not any(falling_edges):
        print "Cannot find rising/falling edges, inductance will be invalid"
        rising_edge_index = 0
        falling_edge_index = len(rising_edges)
    else:        
        # get index of first rising edge
        rising_edge_index = next( i for i,e in enumerate(rising_edges) if e )
        # get index of first falling edge that occurs after rising edge 
        falling_edge_index = next( i for i,e in enumerate(falling_edges[rising_edge_index:]) if e) + rising_edge_index
        # get index of final rising edge that occurs after falling edge
        #final_edge_index = next( i for i,e in enumerate(rising_edges[falling_edge_index:]) if e) + falling_edge_index
            
    #print "rising_edge_index", rising_edge_index
    #print "falling_edge_index", falling_edge_index
    #print "final_edge_index", final_edge_index

    # throw away a certain number of sample for edge
    edge_samples = int(edge_time / xinc)
    print "Throwing away %d samples before falling edge" % edge_samples

    # throw away a certain number of samples after rising edge
    settling_samples = int(settling_time / xinc)
    print "Throwing away first %d samples after rising edge" % settling_samples

    period = len(voltage) * xinc
    print "High pulse period %.2f us" % (period * 1e6)

    voltage = voltage[rising_edge_index+settling_samples:falling_edge_index-edge_samples]
    current = current[rising_edge_index+settling_samples:falling_edge_index-edge_samples]

    # V = L*di/dt    
    di_dt = (current[1:] - current[:-1]) / xinc
    # to make voltage the same like as di/dt, have each sample be average of two nearest neighbors
    voltage = 0.5 * (voltage[1:] + voltage[:-1])
    current = 0.5 * (current[1:] + current[:-1])

    t = pylab.arange(len(voltage)) * xinc * 1e6

    # we only want voltage over inductor, so subtract out resistance
    inductor_voltage = voltage - resistance * current
    print "Average inductor voltage %.2f Volts" % (inductor_voltage.mean())
    
    # L = V/di/dt
    avg_inductance = inductor_voltage.mean()  / di_dt.mean()
    print "Average inductance = %.2f uH" % (avg_inductance * 1e6)

    
    pylab.figure()
    pylab.subplot(2,1,1)
    pylab.plot(t,current)
    pylab.xlabel('time (uSec)')
    pylab.ylabel('current (A)')
    pylab.subplot(2,1,2)
    pylab.plot(t,inductor_voltage, label='voltage over inductor')
    pylab.plot(t,voltage, label='output voltage')
    pylab.xlabel('time (uSec)')
    pylab.ylabel('voltage (V)')
    pylab.legend()
    pylab.show()
    
    
    
    


if __name__ == '__main__':
    main()
