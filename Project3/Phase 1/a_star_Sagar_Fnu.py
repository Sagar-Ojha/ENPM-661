"""
Description: Implementation of A* algorithm to reach the goal position
Date: 3/19/2023
Author: Sagar Ojha (as03050@umd.edu), Fnu Obaid Ur Rahman (obdurhmn@umd.edu)
"""
import heapq as hq
import math as ma
import numpy as np
import time
import matplotlib.pyplot as plt
import cv2

#--------------------------------------------------------------------------------------------------
def generate_child(parent_state, entire_region, step_size, threshold, initial_orientation):
    """! Generates all the children of the given parent.
    This part is just for animation."""
    action_set = [-60, -30, 0, 30, 60]
    child_list = []
    for action in action_set:
        total_angle = action + parent_state[2]
        child_state = (parent_state[0] + step_size * np.cos(np.deg2rad(total_angle)),\
                    parent_state[1] + step_size * np.sin(np.deg2rad(total_angle)))
        child_x = round_nearest(child_state[0], threshold) / threshold
        child_y = round_nearest(child_state[1], threshold) / threshold
        z_index = int(((total_angle - initial_orientation) % 360) / 30)
        if (entire_region[int(child_x)][int(child_y)][z_index] != -1):
            child_list.append((child_x * threshold, child_y * threshold))
    return child_list
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def generate_animation_nodes(visited_list, entire_region, step_size, threshold, initial_orientation):
    """! Creates the parent: child dictionary.
    This part is just for animation."""
    parent_child = {}
    for i in visited_list:
        if (i[-2][0], i[-2][1]) not in parent_child:
            children = generate_child(i[-2], entire_region, step_size, threshold, initial_orientation)
            if (len(children) != 0):
                parent = (i[-1][0], i[-2][1])
                parent_child.update({parent:children}) #{ (p_x, p_y): [(ch1_x,ch1,y), (ch2_x,ch2,y)]}
    return parent_child
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def animate_A_star(obstacle_points, obstacle_points_clearance, obstacle_points_config,\
                    visited_list, optimal_path, entire_region, step_size, threshold, initial_orientation):
    """! Plots the map and animates the node exploration and the optimal path"""

    # print("Length of obstacle points: ", len(obstacle_points))
    # print("Length of obstacle points clearance: ", len(obstacle_points_clearance))
    # print("Length of obstacle points config: ", len(obstacle_points_config))

    plt.xlim([0,600])
    plt.ylim([0,250])
    x_opt = []
    y_opt = []
    for i in range(len(visited_list)):
        x_opt.append(visited_list[i][-1][0])
        y_opt.append(visited_list[i][-1][1])
    plt.scatter(x_opt, y_opt, s = 0.35)
    plt.show()

    display_canvas = np.zeros((251, 601, 3), np.uint8)
    for points in obstacle_points:
        x = int(points[1])
        y = int(points[0])
        display_canvas[(x, y)] = [0, 0, 255]

    for points in obstacle_points_clearance:
        x = int(points[1])
        y = int(points[0])
        display_canvas[(x, y)] = [0, 255, 100]

    for points in obstacle_points_config:
        x = int(points[1])
        y = int(points[0])
        display_canvas[(x, y)] = [255, 100, 0]


    display_canvas = np.flipud(display_canvas)
    cv2.imshow('A*', display_canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    display_canvas_anim = display_canvas.copy()
    node = cv2.VideoWriter('Astar.avi', cv2.VideoWriter_fourcc(*'XVID'), 200, (601, 251))
    parent_child = generate_animation_nodes(visited_list, entire_region, step_size, threshold, initial_orientation)
    # print(len(parent_child))
    # print(parent_child)
    # print(optimal_path)

    for parent, children in parent_child.items():
        for child in children:
            par_x = int(parent[0])
            par_y = int(parent[1])
            par_y = int(250 - par_y)
            child_x = int(child[0])
            child_y = int(child[1])
            child_y = int(250 - child_y)
            cv2.arrowedLine(display_canvas_anim, (par_x, par_y), (child_x, child_y), (0, 255, 50), 1)
            cv2.imshow('Animation', display_canvas_anim)
            node.write(display_canvas_anim)
            if cv2.waitKey(1) & 0xFF == ord('q'):  # Exit if 'q' is pressed
                break

    node.release()
    cv2.destroyAllWindows()

    for i in range(len(optimal_path) - 1):
        x1 = int(optimal_path[i][0])
        y1 = int(optimal_path[i][1])
        y1 = 250 - y1
        x2 = int(optimal_path[i+1][0])
        y2 = int(optimal_path[i+1][1])
        y2 = 250 - y2

        cv2.arrowedLine(display_canvas_anim, (x1, y1), (x2, y2), (0, 0, 255), 2)

    cv2.imshow('shortest path', display_canvas_anim)
    cv2.imwrite('shortest path.jpg', display_canvas_anim)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def generate_path(visited_list):
    """! Backtracking algorithm that finds the optimal path.
    @param visited_list The array of nodes that are explored
    @return The array of coordinates of the optimal path starting from the start to the goal
    """
    node = visited_list[-1]
    node_state = node[-1]
    optimal_path = [node_state]
    while (node_state != (visited_list[0][-1])):
        parent_node = node[-2]
        for i in visited_list:
            if (i[-1] == parent_node):
                node = i
        node_state = node[-1]
        optimal_path.insert(0,node_state)

    return optimal_path
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def round_nearest(num, threshold):
    """! Returns to the nearest multiple of the threshold value used in the tree search"""
    return round(num / threshold) * threshold
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def duplicate_node_check(x, y, child_orientation, initial_orientation, entire_region, child_ctc): # TODO: DEBUG later...
    """!1) Checks if it is a new node or not. No value == New node
        2) If there is CTC value in the matrix, then there is node duplication.
           If CTC in matrix is lower than current CTC, then discard the node and also 
           there is no need to check the unvisited list because it already contains the best CTC.
        3) If the CTC in matrix is higher than current CTC, then check the unvisited list."""

    z_index = int(((child_orientation - initial_orientation) % 360) / 30)

    if z_index == 0:
        left_index = 11
        right_index = 1
    elif z_index == 11:
        left_index = 10
        right_index = 0
    else:
        left_index = z_index - 1
        right_index = z_index + 1
    if ((entire_region[x][y][left_index] == 0) and (entire_region[x][y][z_index] == 0)\
         and (entire_region[x][y][right_index] == 0)):
        entire_region[x][y][z_index] = child_ctc
        entire_region[x][y][left_index] = child_ctc #TODO!! Need to provide parent to the left and right and append in the unvisited_list
        entire_region[x][y][right_index] = child_ctc
        return "New node"

    # If CTC of new node is high, then no need to check the unvisited and visited lists
    elif (entire_region[x][y][z_index] <= child_ctc):
        # print(f'Low')
        entire_region[-1][-1][-1] -= 1
        return "CTC is low"
    # If CTC is low, then check the unvisited list only and update the CTC as well
    elif (entire_region[x][y][z_index] > child_ctc):
        entire_region[x][y][z_index] = child_ctc
        return "CTC is changed"
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def generate_node(parent_ctc, parent_state, unvisited_list, visited_list, entire_region, action,\
                  step_size, threshold, initial_orientation, final_x, final_y):
    """! Generates a new node after performing the action
    @param parent_ctc The parent node state cost to come
    @param parent_state The parent node state (x, y, theta)
    @param unvisited_list The list of nodes to explore
    @param visited_list The list of nodes that are explored
    @param entire_region The entire configuration space map
    @param action The action that is performed to get a new node state
    @param step_size The size of the step that the robot takes
    @param threshold The tree search threshold
    @param initial_orientation Theta at start
    @param final_x, final_y Goal coordinates
    @return None: Just updates the unvisited_list
    """
    total_angle = action + parent_state[2]
    child_state = (parent_state[0] + step_size * np.cos(np.deg2rad(total_angle)),\
                   parent_state[1] + step_size * np.sin(np.deg2rad(total_angle)),\
                   total_angle)
    # print(child_state)

    child_ctc = parent_ctc + step_size
    child_ctg = eucledian_distance(child_state[0], child_state[1], final_x, final_y)
    child_total_cost = child_ctc + child_ctg
    
    child_x = round_nearest(child_state[0], threshold) / threshold
    child_y = round_nearest(child_state[1], threshold) / threshold
    child_orientation = child_state[2]
    child_state = [child_x*threshold, child_y*threshold, child_orientation]
    # print(child_x, child_y, child_orientation)

    if (-1 in entire_region[int(child_x)][int(child_y)]): return # Checks if it is in obstacle space

    flag = duplicate_node_check(int(child_x), int(child_y), child_orientation, initial_orientation,\
                              entire_region, child_ctc)
    if flag == "New node":
        child_node = [child_total_cost, child_ctc, child_ctg, parent_state, child_state]
        hq.heappush(unvisited_list, child_node)
    elif flag == "CTC is changed":
        print("CTC Changed")
        for i in range(len(unvisited_list)):
            if (unvisited_list[i][4] == child_state):
                unvisited_list[i][0] = child_total_cost
                unvisited_list[i][1] = child_ctc
                unvisited_list[i][2] = child_ctg
                unvisited_list[i][3] = parent_state

    return
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def eucledian_distance(x1, y1, x2, y2):
    """! Calculates the distance between two points """
    distance = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
    return distance
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def run_A_star(initial_x, initial_y, final_x, final_y, initial_orientation, final_orientation,\
                obstacle_points, obstacle_points_clearance, obstacle_points_config, threshold,\
                entire_region, step_size):
    """! Finds the shortest path using A* algorithm
    @param intial_x The starting x coordinate
    @param intial_y The starting y coordinate
    @param final_x The final x coordinate
    @param final_y The final y coordinate
    @param initial_orientation The initial orientation of the robot
    @param final_orientation The goal orientation of the robot
    @param obstacle_points Array of all the coordinates in the original obstacle region
    @param obstacle_points_clearance Array of all the coordinates in clearance region
    @param obstacle_points_config Array of all the coordinates in the bloated boundary region
    @param threshold The threshold for tree search
    @return None
    """
    a = 0
    start = time.process_time()
    if (initial_x == final_x) and (initial_y == final_y) and (initial_orientation == final_orientation):
        print(f'The start and the end points as well as the orientation are the same.'\
              ' Rerun the code with different values.')
        return
    # Each node has its state i.e. coordinates and orientaion, cost-to-come, cost-to-go, total-cost,
    # parent node state
    node_state = (initial_x, initial_y, initial_orientation)
    node_ctc = 0
    node_ctg = eucledian_distance(node_state[0], node_state[1], final_x, final_y)
    node_total_cost = node_ctc + node_ctg
    node_parent = (-1, -1, 0)

    # unvisited_list stores the node that is to be explored
    node = [node_total_cost, node_ctc, node_ctg, node_parent, node_state]
    unvisited_list = []
    hq.heappush(unvisited_list, node)
    # visited_list stores the nodes that are explored
    visited_list = []
    # action_set stores the allowed actions that can be performed on the node_state
    action_set = [-60, -30, 0, 30, 60]


    if ((node_ctg <= 1.5)):# and (abs(initial_orientation-final_orientation) <= 30)):
        print(f'Goal reached. Rerun and try different values.')
        return
    else: goal_reached = "no"

    while ((unvisited_list != []) and (goal_reached != "yes")):
        current_node = hq.heappop(unvisited_list) # Node that has lowest total-cost is being explored
        # node_state = current_node[-1]
        # print(current_node)
        visited_list.append(current_node)

        if ((current_node[2] <= 1.5)):# and (abs(current_node[-1][-1]-final_orientation) <= 30)):
            goal_reached = "yes"

        if goal_reached != "yes":
            for action in action_set:
                generate_node(current_node[1], current_node[4], unvisited_list,\
                               visited_list, entire_region, action, step_size, threshold,\
                               initial_orientation, final_x, final_y)
        else:
            print(f'Goal Reached with a cost of {current_node[0]}.')
            optimal_path = generate_path(visited_list)
            print(f'Optimal path found in {time.process_time()-start} seconds.')
            #optimal_path = generate_path(visited_list)
            #print("Optimal path: ", optimal_path)
            animate_A_star(obstacle_points, obstacle_points_clearance, obstacle_points_config,\
                           visited_list, optimal_path, entire_region, step_size, threshold, initial_orientation)
            return
    # print(len(unvisited_list))
    # print(len(visited_list))
    print(f'Goal Not Reached!!!')
    return
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def obstacle_space_points(clearance, threshold):
    """! Creates an array of all the coordinates in the obstacle space
    @param clearance The array with different clearance amounts
    @param threshold The threshold for tree search
    @return Arrays that have all the coordinates of specific regions in the obstacle region
            and a matrix with all the coordinates/map
    """
    obstacle_points, obstacle_points_clearance, obstacle_points_config = [], [], []
    t = threshold
    entire_region = np.zeros([int(600/t), int(250/t), 12], int)

    for c in clearance:
        for x in range(len(entire_region)):
            for y in range(len(entire_region[x])):
                # Inequalities defining the bloated outer walls
                if ((x*t <= c) or (x*t >= (600 - c)) or (y*t <= c) or (y*t >= (250 - c))):
                    if entire_region[x][y][0] == 0:
                        entire_region[x][y] = -1
                        if c == clearance[0]: obstacle_points.append((x*t,y*t))
                        if c == clearance[1]: obstacle_points_clearance.append((x*t,y*t))
                        if c == clearance[2]: obstacle_points_config.append((x*t,y*t))

                # Inequalities defining the bloated rectangular obstacle regions
                elif ((x*t <= (150 + c)) and (x*t >= (100 - c)) and (y*t <= (100 + c)) and (y*t >= 0)):
                    if entire_region[x][y][0] == 0:
                        entire_region[x][y] = -1
                        if c == clearance[0]: obstacle_points.append((x*t,y*t))
                        if c == clearance[1]: obstacle_points_clearance.append((x*t,y*t))
                        if c == clearance[2]: obstacle_points_config.append((x*t,y*t))
                elif ((x*t <= (150 + c)) and (x*t >= (100 - c)) and (y*t >= (150 - c)) and (y*t <= 250)):
                    if entire_region[x][y][0] == 0:
                        entire_region[x][y] = -1
                        if c == clearance[0]: obstacle_points.append((x*t,y*t))
                        if c == clearance[1]: obstacle_points_clearance.append((x*t,y*t))
                        if c == clearance[2]: obstacle_points_config.append((x*t,y*t))

                # Inequalities defining the bloated hexagonal obstacle region
                # The coordinates of the vertices starting from the top and in clockwise direction are:
                # (300, 200), (364.95, 162.5), (394.95, 87.5), (300, 50), (235.05, 87.5), (235.05, 162.5)
                elif ((y*t <= (-0.577 * x*t + 373.21 + c)) and (x*t <= (364.95 + c)) \
                    and (y*t >= (0.577 * x*t - 123.21 - c))  and (y*t >= (-0.577 * x*t + 223.21 - c))\
                    and (x*t >= (235.05 - c)) and (y*t <= (0.577 * x*t + 26.79 + c))):
                    if entire_region[x][y][0] == 0:
                        entire_region[x][y] = -1
                        if c == clearance[0]: obstacle_points.append((x*t,y*t))
                        if c == clearance[1]: obstacle_points_clearance.append((x*t,y*t))
                        if c == clearance[2]: obstacle_points_config.append((x*t,y*t))

                # Inequalities defining the bloated triangular obstacle region
                # The radius for the circles are scaled by 1.07
                # That way the coordinates missed due to truncation are included
                elif (((y*t <= (-2 * x*t + 1145 + c/ma.cos(ma.atan(-2)))) \
                    and (y*t >= (2 * x*t - 895 - c/ma.cos(ma.atan(2)))) and (x*t >= (460 - c)) \
                    and (y*t >= 25) and (y*t <= 225)) or ((((x*t-460)**2 + (y*t-25)**2) <= (c*1.05)**2) \
                                                    or ((x*t-460)**2 + (y*t-225)**2 <= (c*1.05)**2))):
                    if entire_region[x][y][0] == 0:
                        entire_region[x][y] = -1
                        if c == clearance[0]: obstacle_points.append((x*t,y*t))
                        if c == clearance[1]: obstacle_points_clearance.append((x*t,y*t))
                        if c == clearance[2]: obstacle_points_config.append((x*t,y*t))

    return obstacle_points, obstacle_points_clearance, obstacle_points_config, entire_region
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def runner():
    clearance = 5#int(input(f'Enter the clearance amount (mm): '))
    robot_radius = 5#int(input(f'Enter the robot radius (mm): '))
    threshold = 0.5#float(input("Enter the threshold for tree search: "))

    # obstacle_points contains all the coordinates of the original obstacle region
    # obstacle_points_clearance contains the coordinates of the clearance region
    # onstacle_points_config contains the coordinates of the bloated region for the robot radius
    # entire_region contains the entire map
    obstacle_points, obstacle_points_clearance, obstacle_points_config, entire_region = \
        obstacle_space_points([0, clearance, clearance+robot_radius], threshold)

    initial_x, initial_y = 11, 11#map(int,input("Enter the starting x and y position separated by a space: ").split())
    while (((initial_x, initial_y) in obstacle_points) or ((initial_x, initial_y) in obstacle_points_clearance)\
            or((initial_x, initial_y) in obstacle_points_config)):
        print("The values are either in obstacle space or out of bound. Try again!")
        initial_x, initial_y = map(int,input("Enter the starting x and y position separated by a space: ").split())
    final_x, final_y = 510, 215#map(int,input("Enter the final x and y position separated by a space: ").split())
    while (((final_x, final_y) in obstacle_points) or ((final_x, final_y) in obstacle_points_clearance)\
            or ((final_x, final_y) in obstacle_points_config)):
        print("The values are either in obstacle space or out of bound. Try again!")
        final_x, final_y = map(int,input("Enter the final x and y position separated by a space: ").split())

    initial_orientation = 90#int(input('Enter the starting orientation of the robot (in deg): '))
    final_orientation = 30#int(input('Enter the final orientation of the robot (in deg): '))
    step_size = 7#float(input(f'Enter the step size of the robot (1-10): '))
    while (step_size < 1) or (step_size > 10):
        step_size = float(input(f'Enter the step size of the robot (1-10): '))
    
    run_A_star(initial_x, initial_y, final_x, final_y, initial_orientation, final_orientation,\
                obstacle_points, obstacle_points_clearance, obstacle_points_config, threshold,\
                entire_region, step_size)
    print(f'There were {abs(entire_region[-1][-1][-1])} duplicate nodes created.')
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    runner()
    # Start-->(460, 15): Goal-->(450, 15)