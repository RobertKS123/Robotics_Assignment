#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import Empty
from gazebo_msgs.srv import GetModelState
from sensor_msgs.msg import LaserScan

def SLAM():
    return

if __name__ == '__main__':
    try:
        SLAM()
    except rospy.ROSInterruptException:
        pass