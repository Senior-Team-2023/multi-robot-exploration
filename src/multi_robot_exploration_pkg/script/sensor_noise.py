#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
import random

def add_noise(scan):
    ranges = list(scan.ranges)
    for i in range(len(ranges)):
        ranges[i] += random.gauss(0, 0.01)  # 0 mean and 0.01 std deviation
    scan.ranges = ranges
    return scan


def scan_callback_a(scan):
    noisy_scan = add_noise(scan)
    pub_a.publish(noisy_scan)
def scan_callback_b(scan):
    noisy_scan = add_noise(scan)
    pub_b.publish(noisy_scan)
def scan_callback_c(scan):
    noisy_scan = add_noise(scan)
    pub_c.publish(noisy_scan)
if __name__ == '__main__':
    rospy.init_node('noisy_scan')
    rate = rospy.Rate(30)
    sub_a = rospy.Subscriber('/robot/front_laser/scan', LaserScan, scan_callback_a)
    sub_b = rospy.Subscriber('/robot_b/front_laser/scan', LaserScan, scan_callback_b)
    sub_c = rospy.Subscriber('/robot_c/front_laser/scan', LaserScan, scan_callback_c)
    pub_a = rospy.Publisher('/robot/front_laser/noisy_scan', LaserScan, queue_size=10)
    pub_b = rospy.Publisher('/robot_b/front_laser/noisy_scan', LaserScan, queue_size=10)
    pub_c = rospy.Publisher('/robot_c/front_laser/noisy_scan', LaserScan, queue_size=10)
    rospy.spin()