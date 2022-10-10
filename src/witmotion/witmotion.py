#!/usr/bin/env python3

from turtle import pen
import serial
import struct
import datetime

class Witmotion:
    def __init__(self, port='/dev/ttyUSB0', baud=115200):
        self.port = serial.Serial(port, baud, timeout=0.01)
        self.buffer = None
        self.pending_record = {}
        
    def read(self):
        data = self.port.read(2048)
        if len(data) > 0:
            if self.buffer is None:
                self.buffer = data
            else:
                self.buffer += data

        if self.buffer is not None:
            while len(self.buffer) > 0 and self.buffer[0] != 0x55:
                self.buffer = self.buffer[1:]
            if len(self.buffer) == 0:
                self.buffer = None
            else:
                ret = []
                data = self.pending_record
                while len(self.buffer) >= 11:

                    msg = self.buffer[:11]  
                    sum = 0
                    for i in range(10):
                        sum = (sum+msg[i])&0xff
                    if sum != msg[10]:
                        # invalid packet
                        # print('invalid packet')
                        self.buffer = self.buffer[1:]
                    else:
                        self.buffer = self.buffer[11:]
                        try:
                            if msg[1] == 0x50:
                                if 'timestamp' in data:
                                    ret.append(data)
                                    data = {}
                                year, month, day, hour, minute, second, millisecond = struct.unpack('<BBBBBBH', msg[2:10])
                                localtime = datetime.datetime(year+2000, month, day, hour, minute, second, millisecond*1000)
                                now = datetime.datetime.utcnow()
                                tzoffset = localtime - now
                                minutes = round(tzoffset.total_seconds()/60.0)
                                tz = datetime.timezone(datetime.timedelta(minutes=minutes))
                                data['timestamp'] = localtime.replace(tzinfo = tz)
                            elif msg[1] == 0x51:
                                ax,ay,az = struct.unpack('<hhh', msg[2:8])
                                data['acceleration'] = (ax * 16 * 9.8 / 32768.0,
                                                        ay * 16 * 9.8 / 32768.0,
                                                        az * 16 * 9.8 / 32768.0)
                            elif msg[1] == 0x52:
                                avx,avy,avz = struct.unpack('<hhh', msg[2:8])
                                data['angular_velocity'] = (avx * 2000 / 32768.0,
                                                            avy * 2000 / 32768.0,
                                                            avz * 2000 / 32768.0)
                            elif msg[1] == 0x53:
                                r,p,y = struct.unpack('<hhh', msg[2:8])
                                data['roll'] = r * 180 / 32768.0
                                data['pitch'] = p * 180 / 32768.0
                                data['yaw'] = y * 180 / 32768.0
                            elif msg[1] == 0x54:
                                mx,my,mz = struct.unpack('<hhh', msg[2:8])
                                data['magnetic'] = (mx, my, mz)
                            elif msg[1] == 0x56:
                                p,h = struct.unpack('<ii', msg[2:10])
                                data['pressure'] = p
                                data['height'] = h/100.0
                            elif msg[1] == 0x57:
                                lon,lat = struct.unpack('<ii', msg[2:10])
                                lat_deg = int(lat/10000000)
                                lat_min = (abs(lat)%10000000)/100000.0
                                if lat_deg < 0:
                                    lat_min = -lat_min
                                data['latitude'] = lat_deg+lat_min/60.0
                                lon_deg = int(lon/10000000)
                                lon_min = (abs(lon)%10000000)/100000.0
                                if lon_deg < 0:
                                    lon_min = -lon_min
                                data['longitude'] = lon_deg+lon_min/60.0
                            elif msg[1] == 0x58:
                                h,y,v = struct.unpack('<hhi', msg[2:10])
                                data['gps_height'] = h/10.0
                                data['course'] = y/10.0
                                data['speed'] = v/3600.0
                            elif msg[1] == 0x59:
                                q = struct.unpack('<hhhh', msg[2:10])
                                data['quaternion'] = (q[1] / 32768.0,
                                                    q[2] / 32768.0,
                                                    q[3] / 32768.0,
                                                    q[0] / 32768.0)
                            elif msg[1] == 0x5a:
                                sat_count,pdop,hdop,vdop = struct.unpack('<hhhh', msg[2:10])
                                data['satellite_count'] = sat_count
                                data['pdop'] = pdop/32768.0
                                data['hdop'] = hdop/32768.0
                                data['vdop'] = vdop/32768.0
                        except ValueError:
                            data = {}
                ret.append(data)
                data = {}
                self.pending_record = data
                if len(self.buffer) == 0:
                    self.buffer = None
                if len(ret) > 0:
                    return ret
        return None
        
if __name__ == '__main__':
    import sys
    if len(sys.argv) != 3:
        print("usage: witmotion.py port baud (ex: witmotion.py /dev/ttyUSB0 115200)")
    else:
        w = Witmotion(sys.argv[1], int(sys.argv[2]))
        while True:
            data = w.read()
            if data is not None:
                print(len(data),'records')
                for d in data:
                    # for i in d:
                    #     print(' ',i,d[i])
                    try:
                        print(d['timestamp'].isoformat(),d['roll'],d['pitch'],d['yaw'])
                    except KeyError:
                        print(d['timestamp'].isoformat())
            else:
                pass
                #print('no data')
        
