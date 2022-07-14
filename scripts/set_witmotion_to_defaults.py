#!/usr/bin/env python3

import sys
import serial
import struct

if len(sys.argv) != 3:
    print("usage: set_witmotion_to_defaults.py port baud (ex: set_witmotion_to_defaults.py /dev/ttyUSB0 115200)")
else:
    s = serial.Serial(sys.argv[1], int(sys.argv[2]))
    data = struct.pack('<5B', 0xff, 0xaa, 0x00, 0x01, 0x00)
    s.write(data)
    s.flush()
    s.close()
