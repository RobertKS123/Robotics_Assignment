#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

def SLAM():

    while True:
        rospy.init_node('slam_node', anonymous=True)
        rate = rospy.Rate(10)

        # Initialize publishers and messages
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        pub = rospy.Publisher('/my_topic', String, queue_size=10)
        
        # Create the publishers and other required variables
        message = String()
        message.data = "Hello, world!"

        # Publish the message
        pub.publish(message)

        
        # Move the robot forward
        twist_msg = Twist()
        twist_msg.linear.x = 0.1
        velocity_publisher.publish(twist_msg)
        
        # Wait for a small duration
        rospy.sleep(1.0)
        
        # Stop the robot
        twist_msg.linear.x = 0.0
        velocity_publisher.publish(twist_msg)

if __name__ == '__main__':
    try:
        SLAM()
    except rospy.ROSInterruptException:
        pass