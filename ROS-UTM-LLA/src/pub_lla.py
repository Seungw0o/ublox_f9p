#!/usr/bin/env python 

import rospy
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
from std_msgs.msg import Header


class pub_lla:
    def __init__(self):
        rospy.init_node("publish_lla")
        self.pub = rospy.Publisher("/gps/fix", NavSatFix)
        self.sub = rospy.Subscriber("/ublox_gps/fix", NavSatFix, self.cb)
        rospy.spin()

    def cb(self, msg):
        #Header
        seq = msg.header.seq
        stamp = msg.header.stamp
        frame_id = msg.header.frame_id

        #NavSatStatus
        status = msg.status.status
        service = msg.status.service
        
        #NavSatFix
        latitude = msg.latitude
        longitude = msg.longitude
        altitude = msg.altitude
        position_covariance = msg.position_covariance
        position_covariance_type = msg.position_covariance_type

        #Parsing to /gps/fix

        #std_msg/Header
        std_msg_Header = Header()
        std_msg_Header.seq = seq
        std_msg_Header.stamp = stamp
        std_msg_Header.frame_id = frame_id

        #NatSatStatus
        sensor_msgs_NavSatStatus = NavSatStatus()
        sensor_msgs_NavSatStatus.status = status
        sensor_msgs_NavSatStatus.service = service

        #Sensor_msg/NatSatFix
        sensor_msgs_NavSatFix = NavSatFix()
        sensor_msgs_NavSatFix.latitude = latitude
        sensor_msgs_NavSatFix.longitude = longitude
        sensor_msgs_NavSatFix.altitude = altitude
        sensor_msgs_NavSatFix.position_covariance = position_covariance
        sensor_msgs_NavSatFix.position_covariance_type = position_covariance_type
        
        # self.pub.publish(std_msg_Header)
        # self.pub.publish(sensor_msgs_NavSatStatus)
        self.pub.publish(sensor_msgs_NavSatFix)
        

if __name__ == "__main__":
    try:
        pub_lla()
    except:
        pass
