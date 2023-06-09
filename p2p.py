#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Path
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String
import math

# send position not Pose
def euclidean_distance(current, goal):
    return math.sqrt((goal.x - current.x)**2 + (goal.y - current.y)** 2)

def linear_vel(current, goal, constant=1.5):
    return constant * euclidean_distance(current,goal)

def steering_angle(current, goal):
    return math.atan2(goal.y - current.y, goal.x - current.x)

def angular_vel(current, goal, constant=6):
    return constant * (steering_angle(current,goal) - current.theta)

def move_turtle(x,y):
    # Initialize the ROS node
    rospy.init_node('turtlebot_controller', anonymous=True)

    # Create a publisher for the robot's velocity commands
    vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    # Create a rate object to control the loop frequency
    rate = rospy.Rate(10)  # 10 Hz

    # Loop through each point in the path
    goal = Pose()
    goal.x = x
    goal.y = y

    # Create a Twist message to control the robot's velocity
    vel_msg = Twist()

    distance_tolerance = 0.2

    gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    current_pos = gazebo_model_state('mobile_base', 'world')

    # Move the robot towards the current goal point
    while euclidean_distance(current_pos,goal) >= distance_tolerance:
        vel_msg.linear.x = linear_vel(current_pos,goal)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_vel(current_pos,goal)

        vel_pub.publish(vel_msg)

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)

    rospy.spin()


if __name__ == '__main__':
    try:
        x = float(input("Enter the x coord:"))
        y = float(input("Enter the y coord:"))
        move_turtle(x,y)

    except rospy.ROSInterruptException:
        pass