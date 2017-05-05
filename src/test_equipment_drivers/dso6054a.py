""" functions for capturing data from oscilliscope """

from __future__ import print_function
import re

class DSO6054A:
    def __init__(self, dev):
        self.dev = dev
        idn = dev.read("*IDN?")
        if idn is None:
            print(dev.read("*IDN?"))
            raise RuntimeError("Device did not respond to identification request")
        if not re.match('AGILENT TECHNOLOGIES,DSO6054A',idn) and not re.match('AGILENT TECHNOLOGIES,DSO\-X 3034A',idn):
            raise RuntimeError("Bad indentification : ", idn)


    def read_waveform(self, channel, samples=1000):
        """ reads waveform data from oscilliscope using <dev> interface.
        Wavefrom should already be captured on oscilicoe (single).
        <channel> should be integer '1-4'
        returns tuple (x_inc, [data]), where x_inc is timestep of each data sample, and
        data is array of floating point values

        Currently doesn't work if acquisition mode is "Peak Detect"
        """
        w = self.dev.write
        r = self.dev.read

        ch_set = 'CHAN%d'%channel
        w(":waveform:source " + ch_set)
        ch = r(':waveform:source?').strip('\n')
        if ch != ch_set:
            raise RuntimeError("Could not set channel : %s" % (str(ch), str(ch_set)))

        w(":waveform:format ascii")
        format = r(":waveform:format?").strip('\n')
        if format  != "ASC":
            raise RuntimeError("Could not set format : %s" % str(format))

        w(":waveform:points %s" % samples)

        pre = r(':waveform:preamble?')
        if pre is None:
            raise RuntimeError("No response from scope about waveform. Verify scope is 'stopped'.")
        format,typ,points,count,xinc,xorg,xref,yinc,yorg,yref = pre.split(',')

        if int(points) != samples:
            msg = "Wrong number of samples returned : got %s, expected %d" % (points,samples)
            print(msg)
            samples = int(points)
            #raise RuntimeError(msg)

        data = r(':waveform:data?')

        hdr = data[0:2]
        size = int(data[2:10])
        data = data[10:]
        if hdr != "#8":
            raise RuntimeError("Unexpected data header" + hdr)
        if len(data) >= size*2:
            raise RuntimeError("Recv'ed data much bigger that expected size, make sure aquire mode is 'Normal'")
        if size != len(data)-1:
            raise RuntimeError("Data length mismatch : size=%d, len(data)=%d" % (size, len(data)))

        data = [float(d) for d in data.split(',')]
        if len(data) != samples:
            raise RuntimeError("Data sample count mismatch : got %d, expected %d", len(data), points)

        return (float(xinc), data)
