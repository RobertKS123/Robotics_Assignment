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
    pub_cmd_vel = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

    # Create a rate object to control the loop frequency
    rate = rospy.Rate(10)  # 10 Hz

    # Loop through each point in the path
    x = -1
    y = 0
    goal = PoseStamped()
    goal.pose.position.x = x
    goal.pose.position.y = y

    # Create a Twist message to control the robot's velocity
    cmd_vel = Twist()

    print("yes1")
    # Move the robot towards the current goal point
    while not rospy.is_shutdown():
        # Get the current position of the robot
            gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            current_pos = gazebo_model_state('mobile_base', 'world')

            print("yes2")

        # Calculate the distance to the goal point
            distance = math.sqrt((goal.pose.position.x - current_pos.pose.position.x)**2 + (goal.pose.position.y - current_pos.pose.position.y)**2)

        # Check if the goal has been reached
            if distance < 0.1:
                break

            print("yes3")

            # Calculate the linear and angular velocities
            cmd_vel.linear.x = 0.2 * distance
            cmd_vel.angular.z = 1.5 * (math.atan2(goal.pose.position.y - current_pos.pose.position.y, goal.pose.position.x - current_pos.pose.position.x) - current_pos.pose.orientation.w)
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