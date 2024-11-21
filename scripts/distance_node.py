#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from math import sqrt
from geometry_msgs.msg import Twist

# Global variables to store turtles' positions
turtle1_pos = None
turtle2_pos = None
distance_threshold = 2.0  # Stop turtles if distance is less than this

# Callback function to store the positions of turtle1 and turtle2
def pose_callback_turtle1(msg):
    global turtle1_pos
    turtle1_pos = msg

def pose_callback_turtle2(msg):
    global turtle2_pos
    turtle2_pos = msg

# Function to calculate the Euclidean distance between two points
def calculate_distance():
    if turtle1_pos and turtle2_pos:
        x1, y1 = turtle1_pos.x, turtle1_pos.y
        x2, y2 = turtle2_pos.x, turtle2_pos.y
        return sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return 0.0


# Function to stop the turtle
def stop_turtle(turtle_name):
    pub = rospy.Publisher(f'/{turtle_name}/cmd_vel', Twist, queue_size=10)
    move_cmd = Twist()  # Stop command (linear and angular velocity set to 0)
    pub.publish(move_cmd)
    rospy.loginfo(f"{turtle_name} stopped.")

if __name__ == '__main__':
    try:
       pass # distance_monitor()  # Start monitoring the distance
    except rospy.ROSInterruptException:
        pass
