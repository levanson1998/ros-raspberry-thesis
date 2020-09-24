#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
import math


def deg2rad(deg):
    return deg*math.pi/180 

rospy.init_node('lidar_publisher')

scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)

num_readings = 100
laser_frequency = 10

count = 0
r = rospy.Rate(5.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    scan = LaserScan()

    scan.header.stamp = current_time
    scan.header.frame_id = 'laser_frame'
    scan.angle_min = deg2rad(-90)
    scan.angle_max = deg2rad(90)
    scan.angle_increment = deg2rad(180) / num_readings
    scan.time_increment = (5.0 / laser_frequency) / (num_readings)
    scan.range_min = 0.0
    scan.range_max = 100.0

    scan.ranges = []
    scan.intensities = []
    for i in range(0, num_readings):
        scan.ranges.append(1.0 * count)  # fake data
        scan.intensities.append(1)  # fake data
    rospy.loginfo(scan)
    scan_pub.publish(scan)
    count += 1
    r.sleep()

