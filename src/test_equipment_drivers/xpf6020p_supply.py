#!/usr/bin/python
#
# Software License Agreement (BSD License)
#
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
#  * Neither the name of Fetch Robotics, Inc. nor the names of its
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


##\author Camilo F. Buscaron
##\

from test_equipment_drivers.scpi_lxi import LXIDevice

import sys


class xpf60p_supply:

    def __init__(self, address):

        device = LXIDevice(address, port=9221)
        # 9221 is the TCP port specify in the manual of the device

        idn = device.read("*IDN?")
        if idn is None:
            print(device.read("*IDN?"))
            raise RuntimeError("Device did not respond to identification request")
        self._device = device

    def setVoltage(self, voltage):
        self._device.write("V1 " + str(voltage))
        if self.getVoltageSetting() != float(voltage):
            raise RuntimeError("Setting power supply voltage did not work")

    def getVoltageSetting(self):
        """ Get voltage set-point """
        V = self._device.read("V1?")
        return float(V.split(' ')[1])

    def getVoltage(self):
        """ Read output voltage from scope
        output voltage might be lower than setting because of current limiting
        """
        V = self._device.read("V1O?")
        return float(V.split('V')[0])

    def setCurrentLimit(self, current):
        """ Get current limit setting as opposed to output voltage
        which might be different because of current limit"""
        self._device.write("I1 " + str(current))
        if self.getCurrentLimit() != float(current):
            raise RuntimeError("Setting power supply current limit did not work")

    def getCurrentLimit(self):
        I = self._device.read("I1?")
        return float(I.split(' ')[1])

    def setOutput(self, state):
        """ Sets output state, should be true for on and false for disbled """
        self._device.write("OP1 " + str('1' if state else '0'))
        if self.getOutput() != bool(state):
            raise RuntimeError("Setting power supply output state did not work")

    def getOutput(self):
        return int(self._device.read("OP1?"))

    def incrementByStep(self, delta):
        pass
    def decrementByStep(self, delta):
        pass


def main(argv):
    import getopt

    optlist, argv = getopt.gnu_getopt(argv[1:], "h")

    address = argv[0]
    print(address)
    supply1 = xpf60p_supply(address)
    print(supply1)

    print(supply1.setVoltage(9))
    V = supply1.getVoltage()
    print(V)
    print(type(V))

    print(supply1.getCurrentLimit())
    print(supply1.setCurrentLimit(4))

    print("Output state: " + str(supply1.getOutput()))

    print(supply1.setOutput(1))
    print("Output state: " + str(supply1.getOutput()))

    print(supply1.setOutput(0))
    print("Output state: " + str(supply1.getOutput()))

    print(supply1.setVoltage(13.5))
    V = supply1.getVoltage()


if __name__ == '__main__':
    sys.exit(main(sys.argv))
