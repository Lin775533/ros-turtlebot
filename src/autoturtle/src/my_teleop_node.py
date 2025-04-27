#!/usr/bin/env python3

import rospy
import curses
from geometry_msgs.msg import Twist

class TeleopNode:
    def __init__(self):
        # Initialize the node
        rospy.init_node('my_teleop_node', anonymous=True)
        
        # Create a publisher for the cmd_vel topic
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Set the rate
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Print instructions
        rospy.loginfo("my_teleop_node started")
        rospy.loginfo("Use 'w' to move forward, 's' to move backward")
        rospy.loginfo("Use 'a' to rotate left, 'd' to rotate right")
        rospy.loginfo("Press 'q' to quit")
    
    def send_twist(self, linear_vel=0.0, angular_vel=0.0):
        """
        Publish the Twist message with the given velocities
        """
        # Create a Twist message
        move_cmd = Twist()
        
        # Set the linear and angular velocities
        move_cmd.linear.x = linear_vel
        move_cmd.angular.z = angular_vel
        
        # Publish the message
        self.velocity_publisher.publish(move_cmd)

def main():
    # Initialize the TeleopNode
    node = TeleopNode()
    
    # Initialize curses for keyboard input
    stdscr = curses.initscr()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)
    
    try:
        while not rospy.is_shutdown():
            # Get key press
            key = stdscr.getch()
            
            # Process key press
            if key == ord('w'):  # ASCII value for 'w'
                node.send_twist(linear_vel=0.3, angular_vel=0.0)
                rospy.loginfo("Moving forward")
            elif key == ord('s'):  # ASCII value for 's'
                node.send_twist(linear_vel=-0.3, angular_vel=0.0)
                rospy.loginfo("Moving backward")
            elif key == ord('a'):  # ASCII value for 'a'
                node.send_twist(linear_vel=0.0, angular_vel=0.5)
                rospy.loginfo("Rotating left")
            elif key == ord('d'):  # ASCII value for 'd'
                node.send_twist(linear_vel=0.0, angular_vel=-0.5)
                rospy.loginfo("Rotating right")
            elif key == ord('q'):  # ASCII value for 'q'
                rospy.loginfo("Quitting")
                break
            elif key == -1:  # No key pressed
                # Stop the turtle when no key is pressed
                node.send_twist(linear_vel=0.0, angular_vel=0.0)
            
            # Sleep to maintain the rate
            node.rate.sleep()
            
    except Exception as e:
        rospy.logerr(f"Error: {e}")
    finally:
        # Clean up curses
        curses.nocbreak()
        stdscr.keypad(False)
        curses.echo()
        curses.endwin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
