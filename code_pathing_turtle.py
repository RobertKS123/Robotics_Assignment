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

def get_map():
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('slam_package')
    file_path = package_path + '/Robotics_Assignment/map_dialated.png'
    image = Image.open(file_path)
    matrix = np.array(image)
    print(len(matrix))
    return matrix

def convert_coords(coords):
    # Coordinates of the origin of Map A and Map B
    origin_A = (407, 467)
    origin_B = (0, 0)

    # Scaling factor between the two grids
    scaling_factor = 26  # Assuming 20 units in Map A is equal to 1 unit in Map B

    # List of points on the path in Map A
    path_points_A = coords

    # Coordinate transformation
    path_points_B = [(-1 * ((x - origin_A[0]) / scaling_factor),
                    -1 * ((y - origin_A[1]) / scaling_factor))
                    for x, y in path_points_A]


    swapped_points = [(y, x) for x, y in path_points_B]
    # The path points are now transformed to Map B coordinates

    return swapped_points

def convert_coords_reverse(coords):
    origin_A = (467, 407)
    origin_B = (0, 0)

    scaling_factor = 26  # Assuming 20 units in Map A is equal to 1 unit in Map B

    # List of points on the path in Map B
    path_points_B = coords

    # Coordinate transformation
    path_points_A = [(((x * scaling_factor) - origin_A[0]),
                    ((y * scaling_factor ) - origin_A[1]))
                    for x, y in path_points_B]

    swapped_points = np.abs([(y, x) for x, y in path_points_A])

    return swapped_points


def astar_search(matrix, start, goal):
    start = tuple(start)
    goal = tuple(goal)

    rows, cols = matrix.shape
    
    # Define possible movements (up, down, left, right, diagonals)
    movements = [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]
    
    # Create a priority queue (heap) to store nodes to be explored
    open_list = []
    
    # Initialize the start node with a priority of 0 (cost) and add it to the open list
    start_node = (0, start, None)
    heappush(open_list, start_node)
    
    # Create a dictionary to store the cost of reaching each node from the start node
    g_costs = {start: 0}
    
    # Create a dictionary to store the parent node of each explored node
    parents = {}
    
    while open_list:
        # Get the node with the lowest cost (priority) from the open list
        current_cost, current_node, parent_node = heappop(open_list)
        
        # Check if the current node is the goal node
        if current_node == goal:
            # Build the path from the goal node to the start node
            path = []
            while current_node is not None:
                path.append(current_node)
                current_node = parents.get(current_node)  # Use get() to handle missing keys
            return path[::-1]
        
        # Explore the neighboring nodes
        for movement in movements:
            dx, dy = movement
            neighbor = (current_node[0] + dx, current_node[1] + dy)
            
            # Check if the neighbor is within the matrix boundaries
            if neighbor[0] < 0 or neighbor[0] >= rows or neighbor[1] < 0 or neighbor[1] >= cols:
                continue
            
            # Check if the neighbor is an obstacle (white pixel)
            if matrix[neighbor] == 0:
                continue
            
            # Calculate the cost to reach the neighbor from the start node
            neighbor_cost = g_costs[current_node] + 1
            
            # Check if the neighbor has not been visited or the new cost is lower than the previous cost
            if neighbor not in g_costs or neighbor_cost < g_costs[neighbor]:
                # Update the cost to reach the neighbor
                g_costs[neighbor] = neighbor_cost
                
                # Calculate the heuristic (Euclidean distance) from the neighbor to the goal node
                heuristic = np.linalg.norm(np.array(neighbor) - np.array(goal))
                
                # Calculate the priority (cost + heuristic) of the neighbor node
                priority = neighbor_cost + heuristic
                
                # Add the neighbor node to the open list with its priority
                heappush(open_list, (priority, neighbor, current_node))
                
                # Set the parent of the neighbor node
                parents[neighbor] = current_node
    
    # No path found
    return None

def point_line_distance(point, start, end):
    # Calculate the distance between point and the line defined by start and end
    x, y = point
    x1, y1 = start
    x2, y2 = end
    numerator = abs((y2 - y1) * x - (x2 - x1) * y + x2 * y1 - y2 * x1)
    denominator = math.sqrt((y2 - y1)**2 + (x2 - x1)**2)
    return numerator / denominator

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

def get_pos():
    gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    coordinates = gazebo_model_state('mobile_base', 'world')

    return (coordinates.pose.position.y,coordinates.pose.position.x)

def euclidean_distance(current, goal):
    return math.sqrt((goal.x - current.x)**2 + (goal.y - current.y)** 2)

def linear_vel(current, goal, constant=0.2):
    return constant * euclidean_distance(current,goal)

# def steering_angle(current, goal):
#     return math.atan2(goal.y - current.y, goal.x - current.x)

# def angular_vel(current, goal):
#     return 2 * (steering_angle(current.position,goal) - current.orientation.w)

def current_orientation(cur):
    current_orientation_quaternion = (cur.x, cur.y, cur.z, cur.w)

# Convert quaternion to Euler angles
    current_orientation_euler = euler_from_quaternion(current_orientation_quaternion)

# Extract roll, pitch, and yaw angles from Euler angles
    roll = current_orientation_euler[0]
    pitch = current_orientation_euler[1]
    yaw = current_orientation_euler[2]

    return yaw

def rotate_bot(current,goal):
    vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    vel_msg = Twist()

    desired_angle = math.atan2(goal.position.y - current.position.y, goal.position.x - current.position.x)

    angle_diff = 1 - desired_angle - current_orientation(current.orientation)
    while abs(angle_diff) >= 0.02:
        gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        current = gazebo_model_state('mobile_base', 'world').pose

        print ("diff:",angle_diff,"desired", desired_angle,"current orientation:", current_orientation(current.orientation))

        angle_diff = desired_angle - current_orientation(current.orientation)

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angle_diff

        vel_pub.publish(vel_msg)

    vel_msg.angular.z = 0
    vel_pub.publish(vel_msg)

    return True

def move_bot(current,goal):
    vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
    vel_msg = Twist()


    while euclidean_distance(current.position,goal.position) >= 0.2:
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
        x = float(input("x coord:"))
        y = float(input("y coord:"))

        goal = (x,y)

        map = get_map()

        current_pos = get_pos()

        system_converted_coords = convert_coords_reverse([current_pos,goal])

        system_converted_coords = np.around(system_converted_coords,0)

        path = astar_search(map,system_converted_coords[0],system_converted_coords[1])

        if path is not None:

            simple_path = simplify_path(path)

            print(simple_path)

            actual_path = convert_coords(simple_path)

            print(actual_path)

            move_turtle(actual_path)
        
        else:

            print("No path found")

    except rospy.ROSInterruptException:
        pass
