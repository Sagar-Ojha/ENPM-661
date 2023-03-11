"""
Description: Implementation of Dijkstra's algorithm to reach the goal position
Date: 3/11/2023
Author: Sagar Ojha (as03050@umd.edu)
Source Code Link: https://github.com/Sagar-Ojha/ENPM-661/blob/main/Project2/dijkstra_Sagar_Ojha.py
"""
import heapq as hq
import matplotlib.pyplot as plt
import time

#--------------------------------------------------------------------------------------------------
def animate_Dijkstra(obstacle_points, visited_list, optimal_path):
    """! Plots the coordinates that have been explored and the optimal path to the goal
    @param obstacle_points The array of coordinates of the obstacle region
    @param visited_list The array of nodes that are explored
    @param optimal_path The array of coordinates of the optimal path
    @return None
    """
    x_obs = []
    y_obs = []
    for i in obstacle_points:
        x_obs.append(i[0])
        y_obs.append(i[1])
    
    x_vis = []
    y_vis = []
    for i in visited_list:
        x_vis.append(i[2][0])
        y_vis.append(i[2][1])

    x_opt = []
    y_opt = []
    for i in optimal_path:
        x_opt.append(i[0])
        y_opt.append(i[1])

    plt.xlim([0,600])
    plt.ylim([0,250])
    plt.scatter(x_obs, y_obs, marker = "s", s = 0.35, c = 'black')
    for i in range(len(x_vis)):
        plt.scatter(x_vis[i], y_vis[i], marker = "s", s = 0.35, c = 'green')
        plt.pause(0.0001)
    for i in range(len(x_opt)):
        plt.scatter(x_opt[i], y_opt[i], marker = "s", s = 0.35, c = 'red')
        plt.pause(0.0001)
    plt.show()
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def generate_path(visited_list):
    """! Backtracking algorithm that finds the optimal path.
    @param visited_list The array of nodes that are explored
    @return The array of coordinates of the optimal path starting from the start to the goal
    """
    node = visited_list[-1]
    node_state = node[2]
    optimal_path = [node_state]
    while (node_state != (visited_list[0][2])):
        parent_node = node[1]
        for i in visited_list:
            if (i[2] == parent_node):
                node = i
        node_state = node[2]
        optimal_path.insert(0,node_state)

    return optimal_path
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def generate_node(parent_ctc, parent_state, unvisited_list, visited_list, obstacle_points, action):
    """! Generates a new node after performing the action
    @param parent_ctc The parent node state cost to come
    @param parent_state The parent node state
    @param unvisited_list The list of nodes to explore
    @param visited_list The list of nodes that are explored
    @param obstacle_points The list of coordinates inside the obstacle space
    @param action The action that is performed to get a new node state
    @return None: Just updates the unvisited_list
    """
    child_state = (parent_state[0] + action[0], parent_state[1] + action[1])
    if ((action[0] == 0) or (action[1] == 0)):
        child_ctc = parent_ctc + 1
    else:
        child_ctc = parent_ctc + 1.4

    if child_state in obstacle_points: return

    for i in range(len(visited_list)):
        if visited_list[i][2] == child_state: return
    
    node_in_unvisited_list = 0 # counter to check if the child_state is already present in unvisited_list
    for i in range(len(unvisited_list)):
        if (unvisited_list[i][2] == child_state):
            node_in_unvisited_list += 1
            if (unvisited_list[i][0] > child_ctc):
                unvisited_list[i][0] = child_ctc
                unvisited_list[i][1] = parent_state
                unvisited_list[i][2] = child_state

    if node_in_unvisited_list == 0:
        child_node = [child_ctc, parent_state, child_state]
        hq.heappush(unvisited_list, child_node)
    return
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def run_Dijkstra_algorithm(initial_x, initial_y, final_x, final_y, obstacle_points):
    """! Finds the shortest path using Dijkstra's algorithm
    @param intial_x The starting x coordinate
    @param intial_y The starting y coordinate
    @param final_x The final x coordinate
    @param final_y The final y coordinate
    @param obstacle_points Array of all the coordinates in the obstacle region
    @return None
    """
    start = time.process_time()
    if (initial_x == final_x) and (initial_y == final_y):
        print('The start and the end points are the same. Rerun the code with different values.')
        return
    # Each node has its cost-to-come, parent node and the state itself
    node_ctc = 0
    node_parent = (-1, -1)
    node_state = (initial_x, initial_y)

    # unvisited_list stores the node that is to be explored
    node = [node_ctc, node_parent, node_state]
    unvisited_list = []
    hq.heappush(unvisited_list, node)
    # visited_list stores the nodes that are explored
    visited_list = []
    # action_set stores the allowed actions that can be performed on the node_state
    action_set = [(1,0), (-1,0), (0,1), (0,-1), (1,1), (-1,1), (1,-1), (-1,-1)]

    while ((unvisited_list != []) and (node_state != (final_x, final_y))):
        hq.heapify(unvisited_list) # Arranges nodes according to their cost-to-come values
        current_node = hq.heappop(unvisited_list) # Node that has lowest ctc and is being explored
        node_state = current_node[2]
        visited_list.append(current_node)

        if current_node[2] != (final_x, final_y):
            for action in action_set:
                generate_node(current_node[0], current_node[2], unvisited_list,\
                               visited_list, obstacle_points, action)
        else:
            print(f'Goal Reached with a cost of {current_node[0]}.')
            optimal_path = generate_path(visited_list)
            print(f'Optimal path found in {time.process_time()-start} seconds.')
            animate_Dijkstra(obstacle_points, visited_list, optimal_path)
            return

    print(f'Goal Not Reached!!!')
    return
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def obstacle_space_points():
    """! Creates an array of all the coordinates in the bloated obstacle space (with 5 mm clearance)
    @return An array that has all the coordinates in the bloated obstacle region
    """
    c = 5 # clearance in mm
    obstacle_points = [] # Stores the points in the bloated obstacle region
    for x in range(601):
        for y in range(251):
            # Inequalities defining the bloated outer walls
            if ((x <= c) or (x >= (600 - c)) or (y <= c) or (y >= (250 - c))):
                obstacle_points.append((x,y))

            # Inequalities defining the bloated rectangular obstacle regions
            elif ((x <= (150 + c)) and (x >= (100 - c)) and (y <= (100 + c)) and (y >= 0)):
                obstacle_points.append((x,y))
            elif ((x <= (150 + c)) and (x >= (100 - c)) and (y >= (150 - c)) and (y <= 250)):
                obstacle_points.append((x,y))

            # Inequalities defining the bloated hexagonal obstacle region
            # The coordinates of the vertices starting from the top and in clockwise direction are:
            # (300, 200), (364.95, 162.5), (394.95, 87.5), (300, 50), (235.05, 87.5), (235.05, 162.5)
            elif ((y <= (-0.577 * x + 373.21 + c)) and (x <= (364.95 + c)) \
                and (y >= (0.577 * x - 123.21 - c))  and (y >= (-0.577 * x + 223.21 - c))\
                and (x >= (235.05 - c)) and (y <= (0.577 * x + 26.79 + c))):
                obstacle_points.append((x,y))

            # Inequalities defining the bloated triangular obstacle region
            elif ((y <= (-2 * x + 1145 + c)) and (y >= (2 * x - 895 - c)) and (x >= (460 - c))):
                obstacle_points.append((x,y))

    return obstacle_points
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
if __name__ =="__main__":
    obstacle_points = obstacle_space_points() # Stores all the coordinates in the obstacle space
    initial_x, initial_y = input("Enter the starting x and y position separated by a space: ").split()
    while ((int(initial_x), int(initial_y)) in obstacle_points):
        print("The values are either in obstacle space  or out of bound. Try again!")
        initial_x, initial_y = input("Enter the starting x and y position separated by a space: ").split()

    final_x, final_y = input("Enter the final x and y position separated by a space: ").split()
    while ((int(final_x), int(final_y)) in obstacle_points):
        print("The values are either in obstacle space or out of bound. Try again!")
        final_x, final_y = input("Enter the final x and y position separated by a space: ").split()

    run_Dijkstra_algorithm(int(initial_x), int(initial_y), int(final_x), int(final_y), obstacle_points)
    # Start-->(460, 15): Goal-->(450, 15)