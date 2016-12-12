#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan

from DetectLegsFromRaw import collectProcessRawData
def callback(msg):
	collectProcessRawData(msg.ranges, msg.range_min, msg.range_max, msg.angle_min, msg.angle_max, msg.angle_increment)
def main():
	rospy.init_node('laser_scan_listener', anonymous=True)
	rospy.Subscriber("/scan", LaserScan, callback)
	rospy.spin()
if __name__== '__main__':
	main()
