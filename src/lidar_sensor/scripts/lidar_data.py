#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan, PointCloud2
import math
# from rplidar import RPLidar


def deg2rad(deg):
    return deg*math.pi/180

rospy.init_node('lidar_publisher')

scan_pub = rospy.Publisher('scan', LaserScan, queue_size=50)

num_readings = 100
laser_frequency = 10

count = 0
r = rospy.Rate(5.0)

scan = LaserScan()

<<<<<<< HEAD
scan.serial_parityy("/dev/ttyUSB0")
=======
scan.serial_parity("/dev/ttyUSB0")
>>>>>>> e7e1f627f2d07a7e467e6cfe3b818e8986d98c7f
scan.baud_rate(115200)

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

while not rospy.is_shutdown():
    current_time = rospy.Time.now()


    # for i in range(0, num_readings):
    #     scan.ranges.append(1.0 * count)  # fake data
    #     scan.intensities.append(1)  # fake data  
    rospy.loginfo(scan.ranges)
    scan_pub.publish(scan.ranges)
    # count += 1
    r.sleep()
