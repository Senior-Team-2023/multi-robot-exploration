#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
import numpy as np

class NoisyOdom:
    def __init__(self):
        rospy.init_node('noisy_odom', anonymous=True)
        self.sub0 = rospy.Subscriber('/tb3_0/odom', Odometry, self.callback0)
        self.pub0 = rospy.Publisher('/tb3_0/odom_noise', Odometry, queue_size=10)
        self.sub1 = rospy.Subscriber('/tb3_1/odom', Odometry, self.callback1)
        self.pub1 = rospy.Publisher('/tb3_1/odom_noise', Odometry, queue_size=10)
        self.sub2 = rospy.Subscriber('/tb3_2/odom', Odometry, self.callback2)
        self.pub2 = rospy.Publisher('/tb3_2/odom_noise', Odometry, queue_size=10)
        self.sub3 = rospy.Subscriber('/tb3_3/odom', Odometry, self.callback3)
        self.pub3 = rospy.Publisher('/tb3_3/odom_noise', Odometry, queue_size=10)
        self.sub4 = rospy.Subscriber('/tb3_4/odom', Odometry, self.callback4)
        self.pub4 = rospy.Publisher('/tb3_4/odom_noise', Odometry, queue_size=10)

    def callback0(self, msg):
        # Add Gaussian noise to position and orientation data
        noise_pos = np.random.normal(0, 0.1, 3)  # Change 0.1 to desired standard deviation for position noise
        noise_orient = np.random.normal(0, 0.01, 4)  # Change 0.01 to desired standard deviation for orientation noise

        msg.pose.pose.position.x += noise_pos[0]
        msg.pose.pose.position.y += noise_pos[1]
        msg.pose.pose.position.z += noise_pos[2]

        msg.pose.pose.orientation.x += noise_orient[0]
        msg.pose.pose.orientation.y += noise_orient[1]
        msg.pose.pose.orientation.z += noise_orient[2]
        msg.pose.pose.orientation.w += noise_orient[3]

        self.pub0.publish(msg)
    def callback1(self, msg):
        # Add Gaussian noise to position and orientation data
        noise_pos = np.random.normal(0, 0.1, 3)  # Change 0.1 to desired standard deviation for position noise
        noise_orient = np.random.normal(0, 0.01, 4)  # Change 0.01 to desired standard deviation for orientation noise

        msg.pose.pose.position.x += noise_pos[0]
        msg.pose.pose.position.y += noise_pos[1]
        msg.pose.pose.position.z += noise_pos[2]

        msg.pose.pose.orientation.x += noise_orient[0]
        msg.pose.pose.orientation.y += noise_orient[1]
        msg.pose.pose.orientation.z += noise_orient[2]
        msg.pose.pose.orientation.w += noise_orient[3]

        self.pub1.publish(msg)
    def callback2(self, msg):
        # Add Gaussian noise to position and orientation data
        noise_pos = np.random.normal(0, 0.1, 3)  # Change 0.1 to desired standard deviation for position noise
        noise_orient = np.random.normal(0, 0.01, 4)  # Change 0.01 to desired standard deviation for orientation noise

        msg.pose.pose.position.x += noise_pos[0]
        msg.pose.pose.position.y += noise_pos[1]
        msg.pose.pose.position.z += noise_pos[2]

        msg.pose.pose.orientation.x += noise_orient[0]
        msg.pose.pose.orientation.y += noise_orient[1]
        msg.pose.pose.orientation.z += noise_orient[2]
        msg.pose.pose.orientation.w += noise_orient[3]

        self.pub2.publish(msg)
    def callback3(self, msg):
        # Add Gaussian noise to position and orientation data
        noise_pos = np.random.normal(0, 0.1, 3)  # Change 0.1 to desired standard deviation for position noise
        noise_orient = np.random.normal(0, 0.01, 4)  # Change 0.01 to desired standard deviation for orientation noise

        msg.pose.pose.position.x += noise_pos[0]
        msg.pose.pose.position.y += noise_pos[1]
        msg.pose.pose.position.z += noise_pos[2]

        msg.pose.pose.orientation.x += noise_orient[0]
        msg.pose.pose.orientation.y += noise_orient[1]
        msg.pose.pose.orientation.z += noise_orient[2]
        msg.pose.pose.orientation.w += noise_orient[3]

        self.pub3.publish(msg)
    def callback4(self, msg):
        # Add Gaussian noise to position and orientation data
        noise_pos = np.random.normal(0, 0.1, 3)  # Change 0.1 to desired standard deviation for position noise
        noise_orient = np.random.normal(0, 0.01, 4)  # Change 0.01 to desired standard deviation for orientation noise

        msg.pose.pose.position.x += noise_pos[0]
        msg.pose.pose.position.y += noise_pos[1]
        msg.pose.pose.position.z += noise_pos[2]

        msg.pose.pose.orientation.x += noise_orient[0]
        msg.pose.pose.orientation.y += noise_orient[1]
        msg.pose.pose.orientation.z += noise_orient[2]
        msg.pose.pose.orientation.w += noise_orient[3]

        self.pub4.publish(msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = NoisyOdom()
        node.run()
    except rospy.ROSInterruptException:
        pass
