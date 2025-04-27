#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleController:
    def __init__(self):
        # Initialize the node
        rospy.init_node('swim_to_goal', anonymous=True)
        
        # Initialize pose variables
        self.pose = Pose()
        
        # Create publisher for velocity commands
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Create subscriber to receive the turtle's current position
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        
        # Set the rate of the loop
        self.rate = rospy.Rate(10)
        
        # Wait for pose data to be received
        rospy.sleep(1)
    
    def update_pose(self, data):
        """Callback function to update the turtle's pose"""
        self.pose = data
    
    def euclidean_distance(self, goal_x, goal_y):
        """Calculate Euclidean distance between current position and goal"""
        return math.sqrt(pow((goal_x - self.pose.x), 2) + pow((goal_y - self.pose.y), 2))
    
    def angle_to_goal(self, goal_x, goal_y):
        """Calculate angle to goal relative to current orientation"""
        angle = math.atan2(goal_y - self.pose.y, goal_x - self.pose.x)
        return angle - self.pose.theta
    
    def move_to_goal(self):
        """Move the turtle to a user-specified goal"""
        # Infinite loop to keep asking for new goals
        while not rospy.is_shutdown():
            # Ask user for goal position
            try:
                goal_x = float(input("Enter x goal: "))
                goal_y = float(input("Enter y goal: "))
            except ValueError:
                rospy.loginfo("Invalid input. Please enter numeric values.")
                continue
            
            # Proportional controller constants
            K_linear = 1.0  # Linear velocity constant
            K_angular = 4.0  # Angular velocity constant
            
            # Distance tolerance (how close we need to get to the goal)
            distance_tolerance = 0.5
            
            # Create a new Twist message
            vel_msg = Twist()
            
            # Keep moving until we reach the goal
            while self.euclidean_distance(goal_x, goal_y) >= distance_tolerance and not rospy.is_shutdown():
                # Calculate the control inputs
                
                # Linear velocity
                distance = self.euclidean_distance(goal_x, goal_y)
                vel_msg.linear.x = K_linear * distance
                
                # Angular velocity
                angle_error = self.angle_to_goal(goal_x, goal_y)
                vel_msg.angular.z = K_angular * angle_error
                
                # Publish velocity
                self.velocity_publisher.publish(vel_msg)
                
                # Sleep to maintain loop rate
                self.rate.sleep()
            
            # Stop the turtle once we reach the goal
            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            self.velocity_publisher.publish(vel_msg)
            
            # Inform user we've reached the goal
            rospy.loginfo("Reached the goal!")

if __name__ == '__main__':
    try:
        controller = TurtleController()
        controller.move_to_goal()
    except rospy.ROSInterruptException:
        pass