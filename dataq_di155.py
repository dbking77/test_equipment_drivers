#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2016 Fetch Robotics
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

##\author Derek King
##\brief Driver for DATAQ DI-115 : 4-channel data acquisition starter kit

"""
Driver for DATAQ DI-115 : 4-channel data acquisition starter kit

The DI-115 will show up as ACM device, ttyACM#, where # is a
number assigned by the operating system.  It can be found
by looking at dmesg output just after plugging device in,
you will see something like:
  [ 3593.252140] cdc_acm 3-2:1.0: ttyACM0: USB ACM device
In this case the device was assigned to /dev/ttyACM0

Also, you may need to change write/read permissions on
Linux device file, so non-root program can communicate
with sensor.
  sudo chmod 666 /dev/ttyACM0

Usage: %(progname)s [-h] <serial_dev>

Options:
  serial_dev : serial device used to talk to device.
  -h : show this help

Example 1:
  %(progname)s /dev/ttyACM0
"""

from __future__ import print_function

import serial
import sys
import time
import threading

def usage(progname):
    print(__doc__ % vars())

def to_hex(msg):
    return [ ord(char) for char in msg ]


class DataqDi155:
    """
    Driver DATAQ DI-155
    Driver can either be used as interface to send commands and get responss
    directly from device, or be set up to poll analog channel from device in
    background thread

    Example 1 : Sending your own command to device
      dev = DataqDi155('/dev/ttyACM0')
      dev.write('info 0')
      print(dev.read())
      dev.expect_echo("asc")

    Example 2 : Haveing device poll analog channel in background
      dev = DataqDi155('/dev/ttyACM0')
      dev.start(sampling_rate=15, ch1_range=15.0, ch4_range=12.0)
      # do stuff
      print('analog data is', dev.get_analog(1), dev.get_analog(4))
      # wait for new sample
      dev.wait_for_next_sample()
      print('new analog data is', dev.get_analog(1), dev.get_analog(4))
      # Currently you can't do send other commands to device while it is
      # running, you must stop polling first
      dev.stop()
      print('fw version', dev.get_fw_version())
    """

    def __init__(self, dev_acm_name):
        self.ch_data = [None for _ in range(4)]
        self.sample_count = 0
        self.lock = threading.Lock()
        self.read_thread = None
        self.read_thread_should_stop = False

        self.dev = serial.Serial(dev_acm_name, 115200, timeout=0.2, parity=serial.PARITY_NONE, rtscts=0)

    def initialize(self):
        """
        Should be called before using other fuctions,
        but is not absolutely necessary for other functions to work.
        Makes sure that the device stops any streaming data, and
        also verifies that device responds seem to be the correct type
        of of device.
        """
        # try stopping device it left streaming from before
        self._stop()

        # Use info 0 command to poll device for intial response
        for ii in range(3):
            self.write('info 0')
            result = self.read()
            if result == 'info 0 DATAQ':
                break
            elif len(result):
                print("Device returned wrong info0 response :", result)
            else:
                print("Device did not response to info0 request, retrying...")

        # Make sure device is DATAQ DI-155
        self.expect('info 0', 'info 0 DATAQ')
        self.expect('info 1', 'info 1 1550')

        # print firmware version and serial number
        fw_version = self.get_fw_version()
        serial = self.get_serial()
        print("DATAQ : firmware version :", fw_version, " serial ", serial)


    def get_analog(self, channel_num=None):
        """
        Gets most recently read channel data.
        Channel num should be a value between 1 and 4, if if it is not
        specified or None, this will return a list of data for all channels
        If channel wasn't given a voltage range in start() the return value
        will be None
        """
        with self.lock:
            if channel_num is None:
                return self.ch_data
            elif 1 <= channel_num <= len(self.ch_data):
                return self.ch_data[channel_num-1]
            else:
                raise RuntimeError("Invalid analog channel number", channel_num)

    def get_fw_version(self):
        """
        Reads firmware version from device
        """
        self.write('info 2')
        response = self.read()
        args = response.split()
        if (len(args) != 3) or (args[:2] != ['info','2']):
            raise RuntimeError("got invalid response for info2 : " + response)
        fw_version = int(args[2],16)
        fw_version_dot = "%d.%d" % (int(fw_version/100), fw_version%100)
        return fw_version_dot

    def get_serial(self):
        """
        Reads serial number from device.
        """
        self.write('info 6')
        response = self.read()
        args = response.split()
        if (len(args) != 3) or (args[:2] != ['info','6']):
            raise RuntimeError("got invalid response for info6 : " + response)
        serial = args[2]
        return serial

    def expect(self, cmd, expected_response):
        """
        Sends cmd to device and check that response matches
        expected_repsonse
        """
        self.write(cmd)
        response = self.read()
        if response != expected_response:
            errmsg = "Device did not return expected response to "
            errmsg += cmd
            errmsg += "\n expected " + expected_response
            errmsg += "\n got " + response
            raise RuntimeError(errmsg)

    def expect_echo(self, cmd):
        """
        Sends cmd to device and response to match command
        Useful for when sending commands like : srate, slist, asc, float, bin ...
        """
        self.expect(cmd, cmd)

    def start(self, sampling_rate=10, ch1_range=None, ch2_range=None, ch3_range=None, ch4_range=None):
        """
        Configures device analog channels, then starts thread that will process streaming data
        Each analog channel that needs to be used should be give voltage range
        of the signal to be measured, and the code will select the proper voltage range for
        that channels.
        Only channels that are given range values are configured, other channels will not
        be sampled.
        The driver will pick the closest sampling rate to the provided value.
        """
        # calculate closest sampling rate
        srate = int((750000.0 / sampling_rate))
        srate = max(75+1, min(65535-1, srate))
        effective_sampling_rate = 750000.0 / srate
        print("DATAQ : effective sampling rate is %0.2fHz" % effective_sampling_rate)

        self.expect_echo("float")
        self.expect_echo("srate %d" % srate)

        # mapping of max_voltage to gain code
        # ordered starting with min voltage first
        max_voltage_to_gain  = [ (2.500, 7) ]
        max_voltage_to_gain += [ (3.125, 6) ]
        max_voltage_to_gain += [ (5.000, 5) ]
        max_voltage_to_gain += [ (6.250, 4) ]
        max_voltage_to_gain += [ (10.00, 3) ]
        max_voltage_to_gain += [ (12.50, 2) ]
        max_voltage_to_gain += [ (25.00, 1) ]
        max_voltage_to_gain += [ (50.00, 0) ]

        self.sample_to_ch_table = []
        ranges = [ch1_range, ch2_range, ch3_range, ch4_range]
        for ch_num, voltage_range in enumerate(ranges):
            if voltage_range is not None:
                gain_setting = None
                for max_voltage,setting in max_voltage_to_gain:
                    if abs(voltage_range) <= max_voltage:
                        gain_setting = setting
                        break
                if gain_setting is None:
                    raise RuntimeError("No gain setting for voltage range of %f for channel %d" % (max_range, ch_num+1))
                slot_num = len(self.sample_to_ch_table)
                print("DATAQ : channel %d is using slot %d voltage range of %.2f" % (ch_num, slot_num, max_voltage))
                self.expect_echo("slist %d x%04x" % (slot_num, (gain_setting << 8 | ch_num) ))
                self.sample_to_ch_table.append(ch_num)

        self.expect_echo("slist %d xffff" % len(self.sample_to_ch_table))
        # todo spin-up thread to read data
        self.read_thread = threading.Thread(target=self._read_thread_function, name="read_thread")
        self.read_thread.start()

    def stop(self):
        """ Stops the read thread.
        This should be done after start(), because it seems to leave the device
        the active state, and the driver won't be able to reconnect t otherwise
        """
        if self.read_thread:
            self.read_thread_should_stop = True
            self.read_thread.join()

    def wait_for_next_sample():
        """ Waits for next analog sample to be received """
        with self.lock:
            sample_count = self.sample_count
        # TODO use condition variable for this to prevent polling
        while True:
            with self.lock:
                if sample_count != self.sample_count:
                    return
            time.sleep(0.005)

    def flush(self):
        while True:
            response = self.read()
            if not len(response):
                return
            print("Flushed", response)

    def write(self, cmd):
        """ Writes command to device """
        self.dev.write(cmd + '\r')

    def read(self):
        """ Reads response from device strips trailing carriage return from end"""
        response = ""
        while True:
            c = self.dev.read()
            if c:
                response += c
                if c == '\r':
                    break
            else:
                break
        if len(response) and response[-1] != '\r':
            print("DATAQ : non-empty response d/n end with carriage return", response)
            return response
        return response[:-1]


    def _read_thread_function(self):
        try:
            self.expect_echo("start")
            while not self.read_thread_should_stop:
                response = self.read()
                if not len(response):
                    print("DATAQ : No sample")
                    continue
                args = response.split()
                if args[0] != 'sc':
                    print("DATAQ : Response is not a sample", response)
                    continue
                values = args[1:]
                if len(values) != len(self.sample_to_ch_table):
                    print("DATAQ : Response does not have correct number of values", response, values)
                    continue
                ch_data = [None for _ in range(4)]
                for index, value in enumerate(values):
                    voltage = float(value)
                    ch_num = self.sample_to_ch_table[index]
                    ch_data[ch_num] = voltage
                with self.lock:
                    self.sample_count+=1
                    self.ch_data = ch_data
        finally:
            self._stop()

    def _stop(self):
        for _ in range(3):
            self.write("stop")
            for _ in range(3):
                response = self.read()
                if response == "stop":
                    print("DATAQ : got stop response")
                    return
                elif not len(response):
                    print("DATAQ : no response")
                else:
                    print("DATAQ : incorrect response to stop : ", response)
        print("DATAQ : could not stop device")


def main():
    if len(sys.argv) != 2:
        usage(progname)
        sys.exit(1)

    progname = sys.argv[0]
    acm_dev_name = sys.argv[1]
    # TODO use argparse

    dev = DataqDi155(acm_dev_name)

    try:
        dev.initialize()
        try:
            dev.start(ch1_range=15.0, ch3_range=4.0)
            for _ in range(10):
                print(dev.get_analog())
                time.sleep(1)
        finally:
            dev.stop()
    except Exception as ex:
        print("Got exception while running device switching to interactive mode :", str(ex))
    except KeyboardInterrupt:
        print("Caught Ctrl-C, Stopping...")

    line = raw_input("> ")
    while True:
        if line:
            dev.write(line)
        else:
            result = dev.read()
            if result != None:
                print(result)
            else:
                print('<<< NO RESPONSE >>>')
        line = raw_input("> ")


if __name__ == '__main__':
    main()
