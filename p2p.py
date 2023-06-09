#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String
import math

def move_turtle():
    # Initialize the ROS node
    rospy.init_node('turtlebot_controller', anonymous=True)

    # Create a publisher for the robot's velocity commands
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create a rate object to control the loop frequency
    rate = rospy.Rate(10)  # 10 Hz

    # Loop through each point in the path
    point = (-1,1)
    x_goal, y_goal = point

    # Create a Twist message to control the robot's velocity
    cmd_vel = Twist()

    # Move the robot towards the current goal point
    while not rospy.is_shutdown():
        # Get the current position of the robot
            gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            current_pos = coordinates = gazebo_model_state('mobile_base', 'world')

        # Calculate the distance to the goal point
            distance = math.sqrt((x_goal - current_pos[0])**2 + (y_goal - current_pos[1])**2)

        # Check if the goal has been reached
            if distance < 0.1:
                break

            # Calculate the linear and angular velocities
            cmd_vel.linear.x = 0.2 * distance
            cmd_vel.angular.z = 1.5 * (math.atan2(y_goal - current_pos[1], x_goal - current_pos[0]) - current_pos[2])

            # Publish the velocity command
            pub_cmd_vel.publish(cmd_vel)

            # Sleep to control the loop frequency
            rate.sleep()

    # Stop the robot after reaching the goal point
    cmd_vel.linear.x = 0.0
    cmd_vel.angular.z = 0.0
    pub_cmd_vel.publish(cmd_vel)

    # Sleep to allow the robot to stop before moving to the next point
    rospy.sleep(1)


if __name__ == '__main__':
    try:
        move_turtle()

    except rospy.ROSInterruptException:
        pass