#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2015, Derek King
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
##\brief Driver for the Array 371x Electronic Load
##
## Driver is based on 'communication protocol for 371X series DC electronic loads' document from the Array website
##   http://www.array.sh/yqxzcnE.htm
##   http://www.array.sh/download/Communication%20protocol%20for%20electronic%20load.pdf
##
## Actual testing was done with TekPower 3711A DC Electronic Load, which seems to be a rebranded Arrary 3711A
##
## Code was written for python2
## This file contains both a driver class that can be used by other code and
## interactive command prompt.


"""
Interface to Array 371X Electronic Load

Usage: %(progname)s <devname> <address>

Options:
  devname : Serial device that connected to device.  For example /dev/ttyUSB0
  address : Address of device 0-254

Example:
  %(progname)s /dev/ttyUSB0 1

Interative Usage:
  Type command at prompt '>'.
  Valid commands are:
    read, set, local, remote, on, off, start, stop, program

  Interactive commands:
    read : read all available data from device
    set : set new load value needs to extra arguments:
        load_type : either 'current', 'power', 'resistance'
        value : the current, power, or resistance value to set
      before running set command, the load must be put in remote mode,
      by running 'on', 'off', or 'remote' commands
    verify : takes the same arguments and verifies that load is
      running somewhat close to commanded settings
    local : put load in local control mode (controlled from front panel)
    remote : put load in remote control mode (controlled from computer)
    on : enables load, also put load in remote mode
    off : disables load, also put load in remote mode

  Not implemented yet:
    program : load program into scope memory
    start : start running program
    stop : stop running program

Interactive Example : (Reading device )
> read <enter>
{'control': 'remote', 'power': 0.0, 'over_voltage': False, 'load_on': False, 'resistance': 500.0, 'current': 0.0, 'wrong_polarity': False, 'max_power': 310.0, 'over_temp': False, 'max_current': 3000.0, 'over_power': False, 'voltage': 24.413}

Interactive Example : (Voltage measurement from DMM)
> off
> set current 0.1
> on
"""

from __future__ import print_function
import readline
import serial
import sys
import time

def usage(progname):
    print(__doc__ % vars())

class Array371x:
    def __init__(self, devname, address, baud=9600):
        """
        devname : device name of serial port : should be something like /dev/ttyUSB0 in linux
        address : should be address value that eload was configured with (0-254)
        baud : serial baud rate for device : default is 9600
        """
        if (address & 0xFF) != address:
            raise RuntimeError("Invalid address : %d", address)
        self.address = address
        self.dev = serial.Serial(devname, baud, timeout=1)
        self.dev.flushInput()

    def _read(self, cmd=None):
        """
        reads response from device, and verifies command, header, and checksum
        cmd should be command that was sent to device, if not None, the command in the reponse will
        be checked against this value.
        returns read result bytes as list of integers
        for 371x all responses are 26 bytes long, so result will be a list that is length 27
        """
        buf = self.dev.read(26)
        if len(buf) != 26:
            raise RuntimeError("Did not recieve correct number of result bytes, expected 26, got %d" % len(buf))

        # convert buffer into list of integers to make processing easier
        buf = [ord(b) for b in buf]

        if buf[0] != 0xaa:
            raise RuntimeError("Response does not have correct header")
        if buf[1] != self.address:
            raise RuntimeError("Response address does not match")
        if (cmd is not None) and (buf[2] != cmd):
            raise RuntimeError("Response command does not match")
        cksum = self._checksum(buf)
        if buf[25] != cksum:
            raise RuntimeError("Response checksum does not match")
        return buf

    @staticmethod
    def _toVoltage(buf):
        return float( buf[3]<<24 | buf[2]<<16 | buf[1]<<8 | buf[0] ) / 1000.

    @staticmethod
    def _toCurrent(buf):
        return float( buf[1]<<8|buf[0] ) / 1000.

    @staticmethod
    def _fromCurrent(value):
        raw_value = int(round(value * 1000))
        if raw_value < 0 or raw_value > 30000:
            raise RuntimeError("Invalid current value %f" % value)
        return [raw_value&0xFF, (raw_value>>8)&0xFF]

    @staticmethod
    def _toPower(buf):
        return float( buf[1]<<8|buf[0] ) / 10.

    @staticmethod
    def _fromPower(value):
        raw_value = int(round(value * 10))
        if raw_value < 0 or raw_value > 3100:
            raise RuntimeError("Invalid power value %f" % value)
        return [raw_value&0xFF, (raw_value>>8)&0xFF]

    @staticmethod
    def _toResistance(buf):
        return float( buf[1]<<8|buf[0] ) / 100.

    @staticmethod
    def _fromResistance(value):
        raw_value = int(round(value * 100))
        if raw_value < 0 or raw_value > 50000:
            raise RuntimeError("Invalid resistance value %f" % value)
        return [raw_value&0xFF, (raw_value>>8)&0xFF]

    def _write(self, cmd, cmd_data=[]):
        """
        Writes command and command data (if any) to device
        Will pad data to correct length and add header and checksum.
        """
        buf = [0xaa, self.address, cmd] + cmd_data
        buf += [0 for i in range(25-len(buf))]
        buf.append(self._checksum(buf))
        assert(len(buf) == 26)
        if any((b&0x0FF) != b for b in buf):
            raise RuntimeError("Invalid values in command buffer")
        if self._checksum(buf) != buf[-1]:
            raise RuntimeError("Invalid checksum on output")
        self.dev.write(''.join(chr(b) for b in buf))

    @staticmethod
    def _checksum(buf):
        """ sums first 25 values in buffer to compute 8bit checksum """
        return sum(buf[:25])&0xFF

    def read(self):
        """ Reads all state values from electronics load including :
           current, voltage, power, resistance, etc.
        return values in dict()
        """
        self.dev.flushInput()
        cmd = 0x91  # read command
        self._write(cmd)
        buf = self._read(cmd)
        data = dict()
        data['current'] = self._toCurrent(buf[3:])
        data['voltage'] = self._toVoltage(buf[5:])
        data['power'] = self._toPower(buf[9:])
        data['max_current'] = self._toCurrent(buf[11:])
        data['max_power'] = self._toPower(buf[13:])
        data['resistance'] = self._toResistance(buf[15:])
        data['control'] = 'remote' if (buf[17]&(1<<0)) else 'local'
        data['load_on'] = bool(buf[17]&(1<<1))
        data['wrong_polarity'] = bool(buf[17]&(1<<2))
        data['over_temp'] = bool(buf[17]&(1<<3))
        data['over_voltage'] = bool(buf[17]&(1<<4))
        data['over_power'] = bool(buf[17]&(1<<5))
        return data

    def activate(self, enable_load, enable_remote, verify=True):
        cmd = 0x92
        state = (1<<0) if enable_load else 0
        state |= (1<<1) if enable_remote else 0
        self._write(cmd, [state])
        if verify:
            time.sleep(0.1)
            data = self.read()
            control = 'remote' if enable_remote else 'local'
            if data['control'] != control:
                raise RuntimeError("Error verifing new control, expected %s, got %s" % (control, data['control']))
            if data['load_on'] != enable_load:
                raise RuntimeError("Error verify load enabled state, expected %s, got %s" % (str(enable_load), str(data['load_on'])))

    def loadOn(self, verify=True):
        """ Enables active load, also enables remote mode at the same time """
        self.activate(enable_load=True, enable_remote=True)

    def loadOff(self, verify=True):
        """ Disables active load, also enables remote mode at the same time """
        self.activate(enable_load=False, enable_remote=True)

    def local(self, verify=True):
        """ Switch to local mode """
        # when switching to local mode, the enable_load setting
        # doesn't seem to make a differnce
        self.activate(enable_load=False, enable_remote=False)

    def remote(self, verify=True):
        """ Switch to remote (computer controlled) mode """
        # need to read load on/off setting to avoid changing it with this command
        data = self.read()
        self.activate(enable_load=data['load_on'], enable_remote=False)

    def set(self, load_type, value, max_current=30.0, max_power=310.0, new_address=None, verify=True):
        """ Changes load settings
        load_type : 'current', 'power', 'resistance'
        value : load value.  depending on load_type value might be resistance in ohms,
                power in watts, or current in amps.
        maximum_current : set maximum current value.  For 3711A this can be as high as 30amps.
        maximum_power : set maximum power value.  For 3711A this can be as high as 310Wats
        new_address : if specified will set a new address value for load,
                      this will also switch driver over to new address.
        verify : if true, the function will do some extra magic to make sure load actually
                 recieved the requested command
        """
        # Its very important that the set command work reliably, otherwise higher-level
        # tests scripts will have a hard time working correctly.
        # This is even more important for this since there are reports of  
        # of load missing commands on various forums.
        #
        # Unfortunately, the load does not acknowledge commands, so its not
        # possible to simply resend commands, when acknowledgement is not
        # received.
        #
        # Also, the current, voltage, and resistance from the read command, are 
        # measured values, not the commanded values, so if the load is disabled
        # the read values will definately be different than the set values.
        # When the load is on, the actual operating may be slightly different 
        # than commanded values especially if power or voltage limit it hit
        #
        # While it might be possible to repeat commands 2 or 3 time, this does
        # not guarentee that command got through, it just reduces probably of 
        # not getting at least one command (it does not work or detect situations
        # where cable gets unplugged.
        #
        # Instead, this code uses max_power as an acknowledgement, if requested 
        # max power by 0.1Watt.  Since max_power is not a measured value, it
        # can be read back, to verify that the command packet was truely received
        if verify:
            # read current max_power setting
            original_data = self.read()
            if original_data['control'] != 'remote':
                pass
                #raise RuntimeError('can not command load unless in remote mode')
            if (max_power == original_data['max_power']):
                max_power-=0.1

        cmd = 0x90
        cmd_data = self._fromCurrent(max_current) + self._fromPower(max_power)
        if (new_address is not None) and (new_address&0xFF != new_address):
            raise RuntimeError('Invalid address value')
        address = self.address if (new_address is None) else new_address
        cmd_data.append(address)
        if load_type == 'current':
            cmd_data.append(1)
            cmd_data += self._fromCurrent(value)
        elif load_type == 'power':
            cmd_data.append(2)
            cmd_data += self._fromPower(value)
        elif load_type == 'resistance':
            cmd_data.append(3)
            cmd_data += self._fromResistance(value)
        else:
            raise RuntimeError("Invalid load type %s (options are current, power, resistance)" % load_type)
        self._write(cmd,cmd_data)
        
        self.address = address

        if verify:
            time.sleep(0.1)
            data = self.read()
            if data['max_power'] != max_power:
                raise RuntimeError("Command may not have been recieved, read=%f, requested=%f, old=%f" % (data['max_power'], max_power, original_data['max_power']))


    def verify(self, load_type, value):
        """ Verify load is operating near requested set point, raises exception if it is not
        """
        data = self.read()
        if load_type == 'power':
            if abs(data['power'] - value) > 1:
                raise RuntimeError("Expected and measured power are too different, expected %f, measured=%f" % (value, data['power']))
        elif load_type == 'current':
            if abs(data['current'] - value) > 0.05:
                raise RuntimeError("Expected and measured current are too different, expected %f, measured=%f" % (value, data['current']))
        elif load_type == 'resistance':
            if abs(data['resistance'] - value) > 3:
                raise RuntimeError("Expected and measured resistance are too different, expected %f, measured=%f" % (value, data['resistance']))


def main(args):
    progname = args[0]
    if len(args) != 3:
        usage(progname)
        return 1

    devname = args[1]
    address = int(args[2])
    driver = Array371x(devname, address)

    line = raw_input("> ")
    while True:

        if len(line) == 0:
            print("Commands are : set,read,on,off,set,verify,program,start,stop")
        else:
            line = line.split(' ')
            cmd = line[0]
            try:
                if cmd == 'read':
                    print(driver.read())
                elif cmd == 'on':
                    print('enabling load..')
                    driver.loadOn()
                elif cmd == 'off':
                    print('disabling load..')
                    driver.loadOff()
                elif cmd == 'local':
                    print('switching over to local control')
                    driver.local()
                elif cmd == 'remote':
                    print('switching over to remote (PC) control')
                    driver.remote()
                elif cmd in ('set', 'verify'):
                    load_type = line[1]
                    load_value = float(line[2])
                    if cmd == 'set':
                        driver.set(load_type, load_value)
                    else:
                        driver.verify(load_type, load_value)
                else:
                    print("%s is not implemented" % cmd)
            except RuntimeError as ex:
                print("Caught runtime error : ", ex)
        line = raw_input("> ")




if __name__ == "__main__":
    main(sys.argv)
