"""
Description: Implementation of A* algorithm on a differential robot (TurtleBot3 Burger)
Date: 4/07/2023
Author: Sagar Ojha (as03050@umd.edu), Fnu Obaid Ur Rahman (obdurhmn@umd.edu)
"""
import heapq as hq
import math as ma
import numpy as np
import time
import matplotlib.pyplot as plt
import cv2

#--------------------------------------------------------------------------------------------------
def test_plot(matrix_map, threshold, exploration, optimal_path): #TODO: remove afterwards...This is just for quick validations
    # matrix_map is not equivalent to physical map
    # convert the indices in matrix to physical coordinates by multiplying by the threshold amount
    t = threshold
    ob_x = []
    ob_y = []
    cl_x = []
    cl_y = []
    bl_x = []
    bl_y = []

    for i in range(len(matrix_map)):
        for j in range(len(matrix_map[i])):
            if matrix_map[i][j] == -1:
                bl_x.append(i*t)
                bl_y.append(j*t)
            if matrix_map[i][j] == -2:
                cl_x.append(i*t)
                cl_y.append(j*t)
            if matrix_map[i][j] == -3:
                ob_x.append(i*t)
                ob_y.append(j*t)
    
    ex_x = []
    ex_y = []

    for i in range(len(exploration)):
        ex_x.append(exploration[i][4][0])
        ex_y.append(exploration[i][4][1])
    
    opt_x = []
    opt_y = []

    for i in optimal_path:
        opt_x.append(i[0])
        opt_y.append(i[1])
    
    plt.xlim([0,600])
    plt.ylim([0,250])
    plt.scatter(ob_x, ob_y, s = 0.35, color = 'red')
    plt.scatter(cl_x, cl_y, s = 0.35, color = 'green')
    plt.scatter(bl_x, bl_y, s = 0.35, color = 'black')
    plt.scatter(ex_x, ex_y, s = 0.35, color = 'green')
    plt.plot(opt_x, opt_y, color = 'purple')
    # plt.plot([200,x],[50,y], color = 'purple')
    plt.show()
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def animate_A_star(matrix_map, threshold, exploration, optimal_path):

    # Create a named window with the WINDOW_NORMAL flag
    cv2.namedWindow('Animation', cv2.WINDOW_NORMAL)

    # Set the window to be resizable and maximize it
    cv2.resizeWindow('Animation', 800, 600)  # Set an initial size for the window
    cv2.setWindowProperty('Animation', cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
    t = threshold
    display_canvas = np.zeros((251, 601, 3), np.uint8)
    
    #node = [node_total_cost, node_ctc, node_ctg, node_parent, node_state, intermediate_states]
    #node_state = (initial_x, initial_y, initial_orientation)


    for i in range(len(matrix_map)):
        for j in range(len(matrix_map[i])):
            if matrix_map[i][j] == -1:
                x = int(j*t)
                y = int(i*t)
                display_canvas[(x, y)] = [0, 0, 255]
                
            if matrix_map[i][j] == -2:
                x = int(j*t)
                y = int(i*t)
                display_canvas[(x, y)] = [0, 255, 0]

            if matrix_map[i][j] == -3:
                x = int(j*t)
                y = int(i*t)
                display_canvas[(x, y)] = [255, 0, 0]

    display_canvas = np.flipud(display_canvas)
    cv2.imshow('A*', display_canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    display_canvas_anim = display_canvas.copy()
    for i in range(len(exploration)):
        curve = exploration[i][5]
        for j in range(len(curve)-1):
            x1 = int(curve[j][0])
            y1 = int(curve[j][1])
            y1 = 250 - y1
            x2 = int(curve[j+1][0])
            y2 = int(curve[j+1][1])
            y2 = 250 - y2
            cv2.line(display_canvas_anim, (x1, y1), (x2, y2), (0, 0, 255), 2)
        cv2.imshow('Animation', display_canvas_anim)
        if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit if 'q' is pressed
            break

    cv2.destroyAllWindows()

#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def generate_path(visited_list):
    """! Backtracking algorithm that finds the optimal path.
    @param visited_list The array of nodes that are explored
    @return The array of coordinates of the optimal path starting from the start to the goal
    """
    node = visited_list[-1]
    node_state = node[-2]
    optimal_path = [node_state]
    while (node_state != (visited_list[0][-2])):
        parent_node = node[-3]
        for i in visited_list:
            if (i[-2] == parent_node):
                node = i
        node_state = node[-2]
        optimal_path.insert(0,node_state)

    return optimal_path
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def duplicate_node_check(x_mat, y_mat, matrix_map, child_ctc):
    """!
    @param x_mat, y_mat The matrix indices corresponding to the physical coordinates
    @param matrix_map The matrix implementation of the map
    @child_ctc Cost-to-come of the given node
    @return "New node" if cost in the matrix is 0: "CTC is low" if the cost in the matrix is lower 
    than child_ctc: "CTC is changed" if the cost in the matrix is replaced by child_ctc value.
    """
    # 1) Checks if it is a new node or not. No value == New node
    # 2) If there is CTC value in the matrix, then there is node duplication.
    #     If CTC in matrix is lower than current CTC, then discard the node and also 
    #     there is no need to check the unvisited list because it already contains the best CTC.
    # 3) If the CTC in matrix is higher than current CTC, then check the unvisited list.

    if (matrix_map[x_mat][y_mat] == 0):
        matrix_map[x_mat][y_mat] = child_ctc
        return "New node"

    # If CTC of new node is high, then no need to check the unvisited and visited lists
    elif (matrix_map[x_mat][y_mat] <= child_ctc): # This return doesn't have any consequences however
        # print(f'Low')
        return "CTC is low"
    # If CTC is low, then check the unvisited list only and update the CTC as well
    elif (matrix_map[x_mat][y_mat] > child_ctc):
        matrix_map[x_mat][y_mat] = child_ctc
        # print("CTC is changed")
        return "CTC is changed"
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def generate_node(parent_ctc, parent_state, unvisited_list, matrix_map, action, step_size,\
                  threshold, final_x, final_y, wheel_radius, wheel_distance, parent_dict):#TODO: Debug
    """! Generates a new node after performing the action
    @param parent_ctc The parent node state cost to come
    @param parent_state The parent node state (x, y, theta)
    @param unvisited_list The list of nodes to explore
    @param entire_region The entire configuration space map
    @param action The action that is performed to get a new node state
    @param step_size The size of the step that the robot takes
    @param threshold The tree search threshold
    @param final_x, final_y Goal coordinates
    @param wheel_radius, wheel_distance Robot's parameters
    @return None: Just updates the unvisited_list
    """
    # Fine tune dt to overlap the curves generated by similar action such as (50,0) & (100,0)
    rpm1, rpm2 = action[0], action[1]
    dt = 0.1    # Fine tune dt as per need
    t = 0
    x_prev, y_prev, theta_prev = parent_state[0], parent_state[1], parent_state[2]
    intermediate_steps = []
    obstacle = "no"
    child_ctc = parent_ctc
    while ((t < step_size) and (obstacle == "no")):
        x_new = x_prev + ((wheel_radius / 2) * (rpm1 + rpm2) * ma.cos(theta_prev) * dt / 60)
        y_new = y_prev + ((wheel_radius / 2) * (rpm1 + rpm2) * ma.sin(theta_prev) * dt / 60)
        theta_new = theta_prev + ((wheel_radius / wheel_distance) * (rpm2 - rpm1) * dt / 60)
        x_new_mat, y_new_mat = world_to_matrix(x_new, y_new, threshold)
        if (matrix_map[x_new_mat][y_new_mat] < 0):
            obstacle = "yes"
        else:
            child_ctc += eucledian_distance(x_prev, y_prev, x_new, y_new)
            x_prev, y_prev, theta_prev = x_new, y_new, theta_new
            intermediate_steps.append((x_prev, y_prev))
        t += dt
    
    if (t != dt): child_state = (x_prev, y_prev, theta_prev)
    else: return

    # print(child_state)
    
    # Update the parent-child dictionary
    if parent_state in parent_dict:
        parent_dict[parent_state].append((child_state, intermediate_steps))
    else:
        parent_dict[parent_state] = [(child_state, intermediate_steps)]

    child_ctg = eucledian_distance(child_state[0], child_state[1], final_x, final_y)
    child_total_cost = child_ctc + child_ctg

    x_mat, y_mat = world_to_matrix(child_state[0], child_state[1], threshold)
    flag = duplicate_node_check(x_mat, y_mat, matrix_map, child_ctc)

    if flag == "New node":
        child_node = [child_total_cost, child_ctc, child_ctg, parent_state, child_state, intermediate_steps]
        hq.heappush(unvisited_list, child_node)
    elif flag == "CTC is changed":
        # print("CTC Changed")
        for i in range(len(unvisited_list)):
            if (unvisited_list[i][4] == child_state):
                unvisited_list[i][0] = child_total_cost
                unvisited_list[i][1] = child_ctc
                unvisited_list[i][2] = child_ctg
                unvisited_list[i][3] = parent_state
                unvisited_list[i][5] = intermediate_steps

    return
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def eucledian_distance(x1, y1, x2, y2):
    """! Calculates the distance between two points """
    distance = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
    return distance
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def run_A_star(initial_x, initial_y, final_x, final_y, initial_orientation, threshold, matrix_map,\
               wheel_radius, wheel_distance, rpm1, rpm2, step_size):
    """! Finds the shortest path using A* algorithm
    @param intial_x The starting x coordinate
    @param intial_y The starting y coordinate
    @param final_x The final x coordinate
    @param final_y The final y coordinate
    @param initial_orientation The initial orientation of the robot
    @param threshold The threshold for tree search
    @param matrix_map The matrix implementation of the configuration space
    @param wheel_radius, wheel_distance Robot parameters
    @param rpm1, rpm2 Wheel rotations per minutes
    @param step_size The robot's step_size (Given in terms of seconds)
    @return None
    """
    if (initial_x == final_x) and (initial_y == final_y):
        print(f'The start and the end points are the same. Rerun the code with different values.')
        return
    # Each node has its state i.e. coordinates and orientaion, cost-to-come, cost-to-go, total-cost,
    # parent node state and the intermediate coordinates to reach the state
    node_state = (initial_x, initial_y, initial_orientation)
    node_ctc = 0
    node_ctg = eucledian_distance(node_state[0], node_state[1], final_x, final_y)
    node_total_cost = node_ctc + node_ctg
    node_parent = (-1000, -1000, 0) # Just for the parent of the start node
    intermediate_states = [(0,0)]

    # unvisited_list stores the node that is to be explored
    node = [node_total_cost, node_ctc, node_ctg, node_parent, node_state, intermediate_states]
    unvisited_list = []
    hq.heappush(unvisited_list, node)
    # visited_list stores the nodes that are explored
    visited_list = []
    #parent_dict stores the parent state as key and the child state and intermediates steps as value
    parent_dict = {}
    # action_set stores the allowed actions that can be performed on the node_state
    action_set = [[0,rpm1],[rpm1,0],[rpm1,rpm1],[0,rpm2],[rpm2,0],[rpm2,rpm2],[rpm1,rpm2],[rpm2,rpm1]]

    if (node_ctg <= 1.5):
        print(f'Goal reached. Rerun and try different values.')
        return
    else: goal_reached = "no"

    while ((unvisited_list != []) and (goal_reached != "yes")):
        current_node = hq.heappop(unvisited_list) # Node that has lowest total-cost is being explored
        # node_state = current_node[-1]
        # print(current_node)
        visited_list.append(current_node)

        if (current_node[2] <= 1.5):
            goal_reached = "yes"

        if goal_reached != "yes":
            for action in action_set:
                generate_node(current_node[1], current_node[4], unvisited_list, matrix_map, action,\
                              step_size, threshold, final_x, final_y, wheel_radius, wheel_distance, parent_dict)
        else:
            print(f'Goal Reached with a cost of {current_node[0]}.')
            start = time.process_time()
            optimal_path = generate_path(visited_list)
            #print(optimal_path)
            print(f'Optimal path found in {time.process_time()-start} seconds.')
            test_plot(matrix_map, threshold, visited_list, optimal_path)
            animate_A_star(matrix_map, threshold, visited_list, optimal_path)


    return
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def configuration_space(clearance, threshold):
    """! Creates a matrix map of the configuration space
    @param clearance The array with different clearance amounts
    @param threshold The threshold for tree search
    @return A matrix implementing the configuration map
        -3 refers to the coordinate points in the original obstacle (without bloating)
        -2 refers to the clearance region coordinate points
        -1 refers to the bloated region coordinate points
         0 refers to the free space coordinates
    """
    t = threshold
    # Since final orientation doesn't matter, the matrix can be a 2D matrix
    entire_region = np.zeros([int(600/t), int(250/t)], int)

    for c in clearance:
        for x in range(len(entire_region)):
            for y in range(len(entire_region[x])):
                # Inequalities defining the outer walls
                if ((x*t <= c) or (x*t >= (600 - c)) or (y*t <= c) or (y*t >= (250 - c))):
                    entire_region[x][y] -= 1

                # Inequalities defining the rectangular obstacle regions
                elif ((x*t <= (150 + c)) and (x*t >= (100 - c)) and (y*t <= (100 + c)) and (y*t >= 0)):
                    entire_region[x][y] -= 1
                elif ((x*t <= (150 + c)) and (x*t >= (100 - c)) and (y*t <= 250) and (y*t >= (150 -c))):
                    entire_region[x][y] -= 1

                # Inequalities defining the hexagonal obstacle region
                # The coordinates of the vertices starting from the top and in clockwise direction are:
                # (300, 200), (364.95, 162.5), (394.95, 87.5), (300, 50), (235.05, 87.5), (235.05, 162.5)
                elif ((y*t <= (-0.577 * x*t + 373.21 + c)) and (x*t <= (364.95 + c)) \
                    and (y*t >= (0.577 * x*t - 123.21 - c))  and (y*t >= (-0.577 * x*t + 223.21 - c))\
                    and (x*t >= (235.05 - c)) and (y*t <= (0.577 * x*t + 26.79 + c))):
                    entire_region[x][y] -= 1
                
                # Inequalities defining the triangular obstacle region
                elif (((y*t <= (-2 * x*t + 1145 + c/ma.cos(ma.atan(-2)))) \
                    and (y*t >= (2 * x*t - 895 - c/ma.cos(ma.atan(2)))) and (x*t >= (460 - c)) \
                    and (y*t >= 25) and (y*t <= 225)) or ((((x*t-460)**2 + (y*t-25)**2) <= (c)**2) \
                                                    or ((x*t-460)**2 + (y*t-225)**2 <= (c)**2))):
                    entire_region[x][y] -= 1

    return entire_region
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def world_to_matrix(x, y, t): # Unlike in part02, this function doesn't translate the origin
    """! Transforms the coordinate in world/map to matrix coordinates/indices
    @param x, y The coordinates in world frame
    @param t Size of 1 unit distance in map
    @return The coordinates/indices in matrix corresponding to that in the world frame
    """    
    # The corresponding indices in the matrix_map would different owing to the threshold value
    x_mat = round(x / t)
    y_mat = round(y / t)

    return x_mat, y_mat
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def runner():
    # TurtleBot3 Burger dimensions are 13.8 x 17.8 x 19.2 cm^3 with an outer radius of 10.5 cm
    # The wheel radius is 3.3 cm and distance between wheels is 16 cm approximately
    clearance = 1.5#int(input(f'Enter the clearance amount (mm): '))/10
    robot_radius = 10.5
    wheel_radius = 3.3
    wheel_distance = 16
    threshold = 0.5 # Size of a unit length in map. Also, this determines the size of the matrix_map
    step_size = 1#float(input(f'Enter the step size of the robot (in seconds): '))
    rpm1, rpm2 = 50,100#map(int, input("Enter the 2 sets of wheel rpms separated by a space: ").split())

    # matrix_map is a matrix of obstacle region and free space
    matrix_map = configuration_space([clearance + robot_radius, clearance, 0], threshold) # Maintain the sequence

    initial_x, initial_y = 20,20#map(int,input("Enter the starting x and y position in cm separated by a space: ").split())
    initial_x_mat, initial_y_mat = world_to_matrix(initial_x, initial_y, threshold)

    while (matrix_map[initial_x_mat, initial_y_mat] < 0):
        print("The values are either in obstacle space or out of bound. Try again!")
        initial_x, initial_y = map(int,input("Enter the starting x and y position in cm separated by a space: ").split())
        initial_x_mat, initial_y_mat = world_to_matrix(initial_x, initial_y, threshold)

    initial_orientation = 0#int(input('Enter the starting orientation of the robot (in deg): ')) * np.pi / 180

    final_x, final_y = 200,20#map(int,input("Enter the final x and y position in cm separated by a space: ").split())
    final_x_mat, final_y_mat = world_to_matrix(final_x, final_y, threshold)

    while (matrix_map[final_x_mat, final_y_mat] < 0):
        print("The values are either in obstacle space or out of bound. Try again!")
        final_x, final_y = map(int,input("Enter the final x and y position in cm separated by a space: ").split())
        final_x_mat, final_y_mat = world_to_matrix(final_x, final_y, threshold)

    run_A_star(initial_x, initial_y, final_x, final_y, initial_orientation, threshold, matrix_map,\
               wheel_radius, wheel_distance, rpm1, rpm2, step_size)
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    runner()