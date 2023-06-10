#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Path
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String
from PIL import Image
import numpy as np
from heapq import heappop, heappush
import math
import tf
from tf.transformations import euler_from_quaternion
import rospkg

# Loads in a pre proceesed map, couldn't get cv2 to work on ros 
# Map was converted into a binary image and dialated to account for the distance between robots center and the obstacles
def get_map():
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('slam_package')
    file_path = package_path + '/Robotics_Assignment/map_dil2.png'
    image = Image.open(file_path)
    matrix = np.array(image)
    print(len(matrix))
    return matrix

# Converts the coordinates from the gazebo system to the map system
def convert_coords(coords):
    origin_A = (407, 467)
    origin_B = (0, 0)

    # Scaling factor between the two grids
    scaling_factor = 26.0  

    path_points_A = coords

    # Coordinate transformation
    path_points_B = [((-1*(x - origin_A[0]) / scaling_factor),
                    ((y - origin_A[1]) / scaling_factor))
                    for x, y in path_points_A]

    swapped_points = [(y, x) for x, y in path_points_B]
    # The path points are now transformed to Map B coordinates

    return swapped_points

# Converts the coordinates from the map system to the gazebo system
def convert_coords_reverse(coords):
    origin_A = (467, 407)
    origin_B = (0, 0)

    scaling_factor = 26.0

    path_points_B = coords

    path_points_A = [(((x * scaling_factor) + origin_A[0]),
                    ((y * scaling_factor ) - origin_A[1]))
                    for x, y in path_points_B]

    swapped_points = np.abs([(y, x) for x, y in path_points_A])

    return swapped_points

# Pathing algorithm
# Uses map as a martix of obstacles
def astar_search(matrix, start, goal):
    start = tuple([int(x) for x in start])
    goal = tuple([int(x) for x in goal])

    rows, cols = matrix.shape

    # Define possible movements (up, down, left, right, diagonals)
    movements = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]

    # Create a priority queue (heap) to store nodes to be explored
    open_list = []

    # Initialize the start node with a priority of 0 (cost) and add it to the open list
    start_node = (0, start, None)
    heappush(open_list, start_node)

    # Dictionary fot the frontier nodes
    g_costs = {start: 0}

    # Dictionary for the explored nodes
    parents = {}

    while open_list:
        # Get the node with the lowest cost (priority) from the open list
        current_cost, current_node, parent_node = heappop(open_list)

        # Check if the current node is the goal node
        if current_node == goal:
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = parents.get(current_node)
            return path[::-1]
        
        # Explore the neighboring nodes
        for movement in movements:
            dx, dy = movement
            neighbor = (current_node[0] + dx, current_node[1] + dy)

            # Check if the neighbor is not in the obstacles
            if neighbor[0] < 0 or neighbor[0] >= rows or neighbor[1] < 0 or neighbor[1] >= cols:
                continue

            if matrix[neighbor] == 0:
                continue

            # Calculate cost
            neighbor_cost = g_costs[current_node] + 1

            # Check if the neighbor is in frontier or lower cost
            if neighbor not in g_costs or neighbor_cost < g_costs[neighbor]:
                g_costs[neighbor] = neighbor_cost

                # Euclidian distance as heuristic
                heuristic = np.linalg.norm(np.array(neighbor) - np.array(goal))

                # Calculate the priority (cost + heuristic) of the neighbor node
                priority = neighbor_cost + heuristic

                heappush(open_list, (priority, neighbor, current_node))

                # Set the parent of the neighbor node
                parents[neighbor] = current_node
    
    # No path found
    return None

# Calculate the distance between a point and a line defined by start and end
def point_line_distance(point, start, end):
    x, y = point
    x1, y1 = start
    x2, y2 = end
    numerator = abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1)
    denominator = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    return numerator / denominator

# Reduce the number of points in the path to the goal
def simplify_path(points, epsilon=2.0):
    if len(points) < 3:
        return points

    start_point = points[0]
    end_point = points[-1]

    # Find the point with the maximum distance
    max_distance = 0
    max_index = 0
    for i in range(1, len(points) - 1):
        distance = point_line_distance(points[i], start_point, end_point)
        if distance > max_distance:
            max_distance = distance
            max_index = i

    simplified_path = []

    # If the maximum distance is greater than epsilon, recursively simplify the two subpaths
    if max_distance > epsilon:
        left_subpath = simplify_path(points[:max_index + 1], epsilon)
        right_subpath = simplify_path(points[max_index:], epsilon)
        simplified_path = left_subpath[:-1] + right_subpath
    else:
        simplified_path = [start_point, end_point]

    return simplified_path

# Retuns the current position of the robot
def get_pos():
    gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    coordinates = gazebo_model_state('mobile_base', 'world')
    return (coordinates.pose.position.x,coordinates.pose.position.y)

# Get the distance between the robots current position and the next goal
def euclidean_distance(current, goal):
    return math.sqrt((goal.x - current.x)**2 + (goal.y - current.y)** 2)

# Calcualte the linear velocty
def linear_vel(current, goal, constant=0.2):
    return constant * euclidean_distance(current,goal)

# Get the current orientation of the robot using the quaternion
def current_orientation(cur):
    current_orientation_quaternion = (cur.x, cur.y, cur.z, cur.w)

    # Convert quaternion to Euler angles
    current_orientation_euler = euler_from_quaternion(current_orientation_quaternion)

    # Extract roll, pitch, and yaw angles from Euler angles
    roll = current_orientation_euler[0]
    pitch = current_orientation_euler[1]
    yaw = current_orientation_euler[2]

    return yaw

# Rotated the Robot untill it is facing the goal
def rotate_bot(current,goal):
    vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    vel_msg = Twist()

    # Calculate the desired turning angle
    desired_angle = math.atan2(goal.position.y - current.position.y, goal.position.x - current.position.x)
    angle_diff = 1 - desired_angle - current_orientation(current.orientation)

    # Rotate untill within 0.2 rads of the desired angle
    while abs(angle_diff) >= 0.02:
        gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        current = gazebo_model_state('mobile_base', 'world').pose

        angle_diff = desired_angle - current_orientation(current.orientation)

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angle_diff

        vel_pub.publish(vel_msg)

    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)

    return True

# Move the robot towards the current goal
def move_bot(current,goal):
    vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    vel_msg = Twist()

    while euclidean_distance(current.position,goal.position) >= 0.1:
        gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        current = gazebo_model_state('mobile_base', 'world').pose
        m = linear_vel(current.position,goal.position)
        #print("m:",m)
        vel_msg.linear.x = m
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_pub.publish(vel_msg)

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)

    return True


def move_turtle(path):
    # Initialize the ROS node
    rospy.init_node('turtlebot_controller', anonymous=True)

    # Create a rate object to control the loop frequency
    rate = rospy.Rate(10)  # 10 Hz

    for point in path:
        # Loop through each point in the path
        goal = Pose()
        goal.position.x = point[0]
        goal.position.y = point[1]

        gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        current_pos = gazebo_model_state('mobile_base', 'world').pose

        # Move the robot towards the current goal point

        rotate_bot(current_pos,goal)
        move_bot(current_pos,goal)

    print("facken ayy")


if __name__ == '__main__':
    try:
        # Take in the input
        x = float(input("x coord:"))
        y = float(input("y coord:"))
        goal = (x,y)

        # Load the map
        map = get_map()

        # Get the current position
        current_pos = get_pos()

        # Change the position and goal into the maps coordinate system
        system_converted_coords = convert_coords_reverse([current_pos,goal])
        system_converted_coords = np.around(system_converted_coords,0)

        # Calculate path to goal
        path = astar_search(map,system_converted_coords[0],system_converted_coords[1])

        if path is not None:

            simple_path = simplify_path(path)

            print(simple_path)

            actual_path = convert_coords(simple_path)

            print(actual_path)

            # Move the robot to the goal
            move_turtle(actual_path)

        else:

            print("No path found")

    except rospy.ROSInterruptException:
        pass
