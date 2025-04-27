#!/usr/bin/env python3

import rospy
import math
import random
from geometry_msgs.msg import Twist

def swim_two_circles_continuously():
    # Initialize the ROS node
    rospy.init_node('swim_node', anonymous=True)
    
    # Create a publisher for velocity commands
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Set the publishing rate to 10 Hz for smooth motion
    rate = rospy.Rate(10)
    
    # Create a Twist message to hold velocity commands
    vel_msg = Twist()
    
    # Initialize linear velocity randomly ONCE at the start
    linear_vel = random.uniform(0.5, 2.0)  # Random value between 0.5 and 2.0 m/s
    rospy.loginfo("Selected linear velocity: %f m/s", linear_vel)
    
    # Define a fixed radius for the circles
    radius = 1.0  # meters
    
    # Calculate angular velocity based on linear velocity and radius
    angular_vel = linear_vel / radius  # rad/s
    
    # Calculate time to complete one full circle (2π radians)
    time_per_circle = (2 * math.pi) / angular_vel  # seconds
    
    # Total time for a pair of circles (left + right)
    total_time_per_pair = 2 * time_per_circle
    
    # Record the starting time
    start_time = rospy.get_time()
    
    # Main loop to make the turtle swim continuously
    while not rospy.is_shutdown():
        # Get the current time and calculate elapsed time
        current_time = rospy.get_time()
        elapsed_time = current_time - start_time
        
        # Determine the phase within the two-circle cycle
        phase = elapsed_time % total_time_per_pair
        
        # Set angular velocity based on the phase
        if phase < time_per_circle:
            # First circle: turn left with positive angular velocity
            vel_msg.angular.z = angular_vel
        else:
            # Second circle: turn right with negative angular velocity
            vel_msg.angular.z = -angular_vel
        
        # Set the constant linear velocity (unchanged throughout)
        vel_msg.linear.x = linear_vel
        
        # Publish the velocity command
        velocity_publisher.publish(vel_msg)
        
        # Sleep to maintain the 10 Hz rate
        rate.sleep()

if __name__ == '__main__':
    try:
        swim_two_circles_continuously()
    except rospy.ROSInterruptException:
        pass