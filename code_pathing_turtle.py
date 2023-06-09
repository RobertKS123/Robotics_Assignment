#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String
import cv2
import numpy as np
import matplotlib.pyplot as plt
from heapq import heappop, heappush
import math

def get_map():
    image = cv2.imread('base.png')
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, threshold_image = cv2.threshold(gray_image, 150, 255, cv2.THRESH_BINARY)
    binary_image = cv2.bitwise_not(threshold_image)
    #original_matrix = np.array(binary_image)
    dil = cv2.morphologyEx(binary_image,cv2.MORPH_DILATE,np.ones((30,30)))
    binary_image = cv2.bitwise_not(dil)
    matrix = np.array(binary_image)
    return matrix

def convert_coords(coords):
    # Coordinates of the origin of Map A and Map B
    origin_A = (400, 470)
    origin_B = (0, 0)

    # Scaling factor between the two grids
    scaling_factor = 1 / 10  # Assuming 20 units in Map A is equal to 1 unit in Map B

    # List of points on the path in Map A
    path_points_A = coords

    # Coordinate transformation
    path_points_B = [((x - origin_A[0]) * scaling_factor + origin_B[0],
                    (y - origin_A[1]) * scaling_factor + origin_B[1])
                    for x, y in path_points_A]


    swapped_points = [(y, x) for x, y in path_points_B]
    # The path points are now transformed to Map B coordinates

    return swapped_points


def astar_search(matrix, start, goal):
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

def move_turtle(path):
    # Initialize the ROS node
    rospy.init_node('turtlebot_controller', anonymous=True)

    # Create a publisher for the robot's velocity commands
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Create a rate object to control the loop frequency
    rate = rospy.Rate(10)  # 10 Hz

    # Loop through each point in the path
    for point in path:
        x_goal, y_goal = point

        # Create a Twist message to control the robot's velocity
        cmd_vel = Twist()

        # Move the robot towards the current goal point
        while not rospy.is_shutdown():
            # Get the current position of the robot
            current_pos = get_pos()

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
        x = float(input("x coord:"))
        y = float(input("y coord:"))

        goal = (y,x)

        map = get_map()

        current_pos = get_pos()

        path = astar_search(map,current_pos,goal)

        simple_path = simplify_path(path)

        actual_path = convert_coords(simple_path)

        move_turtle(actual_path)

    except rospy.ROSInterruptException:
        pass
