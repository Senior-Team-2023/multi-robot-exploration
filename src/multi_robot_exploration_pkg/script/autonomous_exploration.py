#!/usr/bin/env python3
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

def map_callback(map):
    # Process the occupancy grid here
    # ...
    # Find the furthest movable distance from the map
    max_distance = 0
    goal_x = 0
    goal_y = 0

    for x in range(map.info.width):
        for y in range(map.info.height):
            index = y * map.info.width + x
            if map.data[index] == 0:  # Movable space
                distance = np.sqrt((x - map.info.width/2)**2 + (y - map.info.height/2)**2)
                if distance > max_distance:
                    max_distance = distance
                    goal_x = x
                    goal_y = y

    # Convert from map pixel coordinates to map frame coordinates
    goal_x = 10
    goal_y = 10
    # goal_x = goal_x * map.info.resolution + map.info.origin.position.x
    # goal_y = goal_y * map.info.resolution + map.info.origin.position.y

    # Set the goal based on the furthest movable distance
    goal = PoseStamped()
    goal.header.frame_id = "map"
    goal.pose.position.x = goal_x
    goal.pose.position.y = goal_y
    goal.pose.orientation.w = 1.0
    # Get the current time
    now = rospy.Time.now()
    goal.header.stamp.secs = now.secs
    goal.header.stamp.nsecs = now.nsecs
    # Publish the goal to move_base
    goal_publisher.publish(goal)

if __name__ == '__main__':
    rospy.init_node('exploration')
    rate = rospy.Rate(30)
    map_subscriber = rospy.Subscriber('/robot/map', OccupancyGrid, map_callback, queue_size=2)
    goal_publisher = rospy.Publisher('/robot/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.spin()
