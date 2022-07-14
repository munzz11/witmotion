#!/usr/bin/env python3

from witmotion import witmotion
import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from sensor_msgs.msg import Imu
from sensor_msgs.msg import TimeReference
from geometry_msgs.msg import TwistStamped

import math

rospy.init_node('witmotion')

port = rospy.get_param('~port', '/dev/ttyUSB0')
baud = rospy.get_param('~baud', 9600)
    
frame_id = rospy.get_param('~frame_id', 'witmotion')
    
position_pub = rospy.Publisher('~position',NavSatFix,queue_size=10)
timeref_pub = rospy.Publisher('~time_reference',TimeReference,queue_size=10)
orientation_pub = rospy.Publisher('~orientation',Imu,queue_size=10)
velocity_pub = rospy.Publisher('~velocity',TwistStamped,queue_size=10)

wm = witmotion.Witmotion(port, baud)    

navsat_status = None

while not rospy.is_shutdown():
    data = wm.read()
    if data is not None:
        now = rospy.get_rostime()
        for d in data:
            # time
            #print (d)
            if 'timestamp' in d:
                sensor_time = rospy.Time.from_sec(d['timestamp'].timestamp())
                tref = TimeReference()
                tref.header.stamp = now
                tref.time_ref = sensor_time
                tref.source = frame_id
                timeref_pub.publish(tref)

                if 'satellite_count' in d:
                    navsat_status = NavSatStatus()
                    navsat_status.service = NavSatStatus.SERVICE_GPS
                    navsat_status.status = NavSatStatus.STATUS_NO_FIX
            
                    if d['satellite_count'] > 2:
                        navsat_status.status = NavSatStatus.STATUS_FIX

                        try:
                            nsf = NavSatFix()
                            nsf.status = navsat_status
                            nsf.header.stamp = sensor_time
                            nsf.header.frame_id = frame_id
                            nsf.latitude = d['latitude']
                            nsf.longitude = d['longitude']
                            nsf.altitude = d['gps_height']
                            position_pub.publish(nsf)
                        except KeyError:
                            pass
            
                if 'quaternion' in d:

                    try:

                        imu = Imu()                  
                        imu.header.stamp = sensor_time
                        imu.header.frame_id = frame_id
                        
                        imu.orientation.x = d['quaternion'][0]
                        imu.orientation.y = d['quaternion'][1]
                        imu.orientation.z = d['quaternion'][2]
                        imu.orientation.w = d['quaternion'][3]
                        imu.angular_velocity.x = math.radians(d['angular_velocity'][0])
                        imu.angular_velocity.y = math.radians(d['angular_velocity'][1])
                        imu.angular_velocity.z = math.radians(d['angular_velocity'][2])
                        imu.linear_acceleration.x = d['acceleration'][0]
                        imu.linear_acceleration.y = d['acceleration'][1]
                        imu.linear_acceleration.z = d['acceleration'][2]
                        

                        orientation_pub.publish(imu)

                        ts = TwistStamped()
                        ts.header.stamp = sensor_time
                        ts.header.frame_id = frame_id
                        yaw_rad = math.radians(d['course'])
                        ts.twist.linear.x = d['speed']*math.cos(yaw_rad)
                        ts.twist.linear.y = d['speed']*math.sin(yaw_rad)
                        
                        ts.twist.angular = imu.angular_velocity
                        velocity_pub.publish(ts)
                    except KeyError:
                        pass


