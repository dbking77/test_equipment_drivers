#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Derek King
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
##\brief Driver for the Futek USB210 Torque Sensor Amplifier

from __future__ import print_function

import serial
import time
import threading

class FutekUSB210:
    """ Driver for FutekUSB210. Might also work for USB220 and USB320 """

    def __init__(self, dev_name):
        """ dev_name Name of serial device that USB adapter shows up as:
        often /dev/ttyUSB0 in Linux
        """        
        # USB210 uses 9600 baud 8N1
        self.dev = serial.Serial(dev_name, 9600, bytesize=8, parity='N', stopbits=1, xonxoff=0, rtscts=0, timeout=1.0)

        self.value = None # most recently read torque value
        self.callback = None
        self._stop = False        
        self.error = ''
        self.last_error = ''
        
    def getValue(self):
        return self.value

    def setCallback(self, callback_func):
        """ Set a function that will be called when new sample data is read"""
        self.callback = callback_func

    def readLoop(self):
        """ Runs loop that continously reads new sensor data.  
        New data can be access synchronously by setting a callback function.  
        The data can also be access asynchronously by calling getValue when needed
        This will not return until stop() is called.  
        Use startReadLoopInThread to run this loop in background thread
        """
        while not self._stop:
            try:
                self.value = self.read()
                if self.callback is not None:
                    self.callback(self.value)                    
            except RuntimeError, e:
                self.error = str(e)
                self.last_error = self.error

    def startReadLoopInThread(self):
        """ Start thread to run readLoop()"""
        self.thread = threading.Thread(group=None, target=self.readLoop)
        self.thread.start()

    def stop(self):
        """ While stop readLoop (or thread)"""
        self._stop = True

    def read(self):
        """ reads a single data sample and returns value as float
        Will raise exception is there is a read timeout or the read value cannot be parsed
        """
        data_str = self.dev.readline()
        data = data_str.split()
        if len(data) != 2:
            raise RuntimeError("Cannot parse read " + str(data))
        if data[1] != 'N-m':
            raise RuntimeError("Unexpected unit '%s'" % data[1] + data_str) 
        try:
            value = float(data[0])
        except ValueError:
            raise RuntimeError("Cannot convert '%s' to float" % data[0])
        return value
                
zero = None
def callback(value):
    global zero
    if zero is None:
        zero = value
    print(value, value-zero)


if __name__ == '__main__':    
    import sys
    sensor = FutekUSB210(sys.argv[1])
    sensor.setCallback(callback)
    #sensor.startReadLoopInThread() 
    sensor.readLoop()
    #print(sensor.read())
    #time.sleep(10)
    #sensor.stop()
    #print("done")
    
