#!/usr/bin/env python

import rospy
from turtlesim.srv import Spawn
from geometry_msgs.msg import Twist

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



def move_turtle(turtle_name, linear_velocity, angular_velocity):
        
    # Initialize the publisher to send commands to the selected turtle
    pub = rospy.Publisher(f"/{turtle_name}/cmd_vel", Twist , queue_size= 10 )

    # Create the Twist message to control the turtle's velocity
    move_cmd = Twist()
    move_cmd.linear.x = linear_velocity
    move_cmd.angular.z = angular_velocity

    # Set the rate to control the loop frequency (1 Hz is 1 second)
    rate= rospy.Rate(1)
    rospy.loginfo(f"Moving {turtle_name} with linear velocity {linear_velocity} and angular velocity {angular_velocity}")

    # Publish the velocity for 1 second
    for _ in range(1):

        pub.publish(move_cmd)
        rate.sleep()

    # Stop the turtle after 1 second by publishing zero velocities
    move_cmd.linear.x   = 0
    move_cmd.angular.z  = 0
    pub.publish(move_cmd)
    rospy.loginfo(f"{turtle_name} has stopped")





def get_user_input():
    while not rospy.is_shutdown():
        print('select the turtle : 1 or 2 ')
        turtle_name = input()

        if turtle_name not in ['turtle1', 'turtle2']:
            print('turtle name is incorrect')
            continue

        print(f"Enter the linear velocity for {turtle_name} (in meters per second):")
        linear_velocity = float(input())

        print(f"enter angular velocity for {turtle_name}")
        angular_velocity = float(input())

        #for moving the selected turtle
        move_turtle(turtle_name , linear_velocity  ,angular_velocity)



if __name__ == '__main__':
    try:
        spawn_turtle()  # Call the function to spawn the turtle
        get_user_input() # call the user input function 
    except rospy.ROSInterruptException:
        pass
