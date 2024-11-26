#!/usr/bin/env python

import rospy
from turtlesim.msg import Pose
from std_msgs.msg import Float32
from math import sqrt
from geometry_msgs.msg import Twist

# Global variables to store turtles' positions
turtle1_pos = None
turtle2_pos = None
distance_threshold = 5.0  # Stop turtles if distance is less than this or threshhold

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

# Function to monitor the turtles' distance and stop if too close or near boundaries
def distance_monitor():
    rospy.init_node('distance_node', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback_turtle1)
    rospy.Subscriber('/turtle2/pose', Pose, pose_callback_turtle2)

    distance_pub = rospy.Publisher('/turtle_distance', Float32, queue_size=10)

    rate = rospy.Rate(1)  # Check every second
    while not rospy.is_shutdown():
        if turtle1_pos and turtle2_pos:
            distance = calculate_distance()
            distance_pub.publish(distance)  # Publish the distance

            # Stop turtles if they are too close
            if distance < distance_threshold:
                rospy.loginfo("Turtles are too close! Stopping turtles.")
                stop_turtle('turtle1')
                stop_turtle('turtle2')

            # Check if turtles are too close to boundaries (1.0 or 10.0)
            if (turtle1_pos.x < 1.0 or turtle1_pos.x > 10.0 or 
                turtle1_pos.y < 1.0 or turtle1_pos.y > 10.0):
                rospy.loginfo("Turtle1 is too close to boundary! Stopping turtle1.")
                stop_turtle('turtle1')

            if (turtle2_pos.x < 1.0 or turtle2_pos.x > 10.0 or 
                turtle2_pos.y < 1.0 or turtle2_pos.y > 10.0):
                rospy.loginfo("Turtle2 is too close to boundary! Stopping turtle2.")
                stop_turtle('turtle2')  

        rate.sleep()

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
