""" functions for capturing data from oscilliscope """

import re

class DSO6054A:
    def __init__(self, dev):
        self.dev = dev
        idn = dev.read("*IDN?")
        if idn is None:
            raise RuntimeError("Device did not respond to identification request")
        if not re.match('AGILENT TECHNOLOGIES,DSO6054A',idn) and not re.match('AGILENT TECHNOLOGIES,DSO\-X 3034A',idn):
            raise RuntimeError("Bad indentification : ", idn)
        

    def read_waveform(self, channel, samples=1000):
        """ reads waveform data from oscilliscope using <dev> interface.  
        Wavefrom should already be captured on oscilicoe (single).  
        <channel> should be integer '1-4'
        returns tuple (x_inc, [data]), where x_inc is timestep of each data sample, and 
        data is array of floating point values
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

        pre = r('waveform:preamble?')
        if pre is None:
            raise RuntimeError("No response from scope about waveform, is scope 'stopped'")
        format,typ,points,count,xinc,xorg,xref,yinc,yorg,yref = pre.split(',')

        if int(points) != samples:
            msg = "Wrong number of samples returned : got %s, expected %d" % (points,samples)
            print msg
            samples = int(points)
            #raise RuntimeError(msg)

        data = r(':waveform:data?')

        hdr = data[0:2]
        size = int(data[2:10])
        data = data[10:]
        if hdr != "#8":
            raise RuntimeError("Unexpected data header" + hdr)
        if size != len(data)-1:
            raise RuntimeError("Data length mismatch")

        data = [float(d) for d in data.split(',')]
        if len(data) != samples:
            raise RuntimeError("Data sample count mismatch : got %d, expected %d", len(data), points)

        return (float(xinc), data)


    
    
    

    
