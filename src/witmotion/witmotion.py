#!/usr/bin/env python3

import serial
import struct

class Witmotion:
    def __init__(self, port='/dev/ttyUSB0', baud=9600):
        self.port = serial.Serial(port, baud, timeout=0.1)
        self.buffer = None
        
    def read(self):
        while True:
            data = self.port.read(100)
            if len(data) == 0:
                break
            if self.buffer is None:
                self.buffer = data
            else:
                self.buffer += data
        ret = self.buffer
        self.buffer = None
        return ret
        
if __name__ == '__main__':
    import sys
    if len(sys.argv) != 3:
        print("usage: witmotion.py port baud (ex: witmotion.py /dev/ttyUSB0 115200)")
    else:
        w = Witmotion(sys.argv[1], int(sys.argv[2]))
        while True:
            data = w.read()
            print(len(data),'bytes')
        
