#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String

def compute_path(current_pose,target_pose):
    path = Path()
    path.header.frame_id = "map"

    # Create the start pose
    start_pose = PoseStamped()
    start_pose.pose = current_pose.pose
    path.poses.append(start_pose)

    # Create the target pose
    target_pose.header.frame_id = "map"
    path.poses.append(target_pose)

    return path

def path_planning(x,y):
    current_pose = PoseStamped()
    target_pose = PoseStamped()

    target_pose.pose.position.x = x
    target_pose.pose.position.y = y

    rospy.init_node('path_planning_node', anonymous=True)
    rate = rospy.Rate(10)

    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)


    while not rospy.is_shutdown():
        # Get the current position from Gazebo
        try:
            gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            coordinates = gazebo_model_state('mobile_base', 'world')
            current_pose.pose.position.x = coordinates.pose.position.x
            current_pose.pose.position.y = coordinates.pose.position.y
            print(current_pose.pose.position.x,current_pose.pose.position.y)
        except rospy.ServiceException as e:
            rospy.logerr('Service call failed: {}'.format(e))

        # Perform path planning algorithm to compute the path
        path = compute_path(current_pose, target_pose)

        print(path)
        # Publish the computed path
        path_pub.publish(path)

        # Move the TurtleBot along the path (simple example)
        if len(path.poses) > 0:
            goal_pub.publish(path.poses[-1])

        rate.sleep()

if __name__ == '__main__':
    try:
        x = float(input("x coord:"))
        y = float(input("y coord:"))
        path_planning(x,y)
    except rospy.ROSInterruptException:
        pass