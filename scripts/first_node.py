#!/usr/bin/env python

import rospy
from turtlesim.srv import Spawn

def spawn_turtle():
    # Initialize the ROS node
    rospy.init_node('ui_node', anonymous=True)
    
    # Wait for the spawn service to be available
    rospy.wait_for_service('/spawn')
    
    try:
        # Create a service proxy for the /spawn service
        spawn_service = rospy.ServiceProxy('/spawn', Spawn)
        
        # Spawn a new turtle named 'turtle2' at (5.0, 5.0) with orientation 0
        spawn_service(3.0, 3.0, 0.0, 'turtle2')
        
        rospy.loginfo("Turtle2 spawned at (5.0, 5.0) with orientation 0.")
        
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        spawn_turtle()  # Call the function to spawn the turtle
    except rospy.ROSInterruptException:
        pass
