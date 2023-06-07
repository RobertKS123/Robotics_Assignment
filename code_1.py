#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_robot():
    # Initialize the ROS node
    rospy.init_node('turtlebot_controller', anonymous=True)

    # Create a publisher for publishing velocity commands
    velocity_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    # Set the rate at which to publish the velocity commands (in Hz)
    rate = rospy.Rate(10)

    # Create a Twist message
    velocity_msg = Twist()

    # Set the linear and angular velocities
    velocity_msg.linear.x = 1   # Adjust the linear velocity as needed
    velocity_msg.angular.z = 1  # Adjust the angular velocity as needed

    # Keep publishing the velocity commands until Ctrl+C is pressed
    while not rospy.is_shutdown():
        # Publish the velocity command
        velocity_pub.publish(velocity_msg)
        
        # Print a message to indicate that the velocity command is published
        rospy.loginfo("Velocity command published!")

        # Sleep for a while to maintain the desired publishing rate
        rate.sleep()

        # Stop the robot after a certain time (e.g., 5 seconds)
        rospy.sleep(5)
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        velocity_pub.publish(velocity_msg)

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass