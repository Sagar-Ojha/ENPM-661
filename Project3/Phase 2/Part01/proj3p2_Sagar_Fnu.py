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
def test_plot(matrix_map, threshold, x, y): #TODO: remove afterwards...This is just for quick validations
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
    
    plt.xlim([0,600])
    plt.ylim([0,250])
    plt.scatter(ob_x, ob_y, s = 0.35, color = 'red')
    plt.scatter(cl_x, cl_y, s = 0.35, color = 'green')
    plt.scatter(bl_x, bl_y, s = 0.35, color = 'black')
    plt.plot([200,x*t],[50,y*t], color = 'purple')
    plt.show()
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
    x_mat = int(x / t)
    y_mat = int(y / t)

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
    step_size = 1#float(input(f'Enter the step size of the robot (1-10): ')) #TODO: edit later..........
    rpm1, rpm2 = 200,200#map(int, input("Enter the 2 sets of wheel rpms separated by a space: ").split())

    # matrix_map is a matrix of obstacle region and free space
    matrix_map = configuration_space([clearance + robot_radius, clearance, 0], threshold) # Maintain the sequence

    initial_x, initial_y = 20,20#map(int,input("Enter the starting x and y position in cm separated by a space: ").split())
    initial_x, initial_y = world_to_matrix(initial_x, initial_y, threshold)

    while (matrix_map[initial_x, initial_y] < 0):
        print("The values are either in obstacle space or out of bound. Try again!")
        initial_x, initial_y = map(int,input("Enter the starting x and y position in cm separated by a space: ").split())
        initial_x, initial_y = world_to_matrix(initial_x, initial_y, threshold)

    initial_orientation = 0#int(input('Enter the starting orientation of the robot (in deg): '))

    final_x, final_y = 20,20#map(int,input("Enter the final x and y position in cm separated by a space: ").split())
    final_x, final_y = world_to_matrix(final_x, final_y, threshold)

    while (matrix_map[final_x, final_y] < 0):
        print("The values are either in obstacle space or out of bound. Try again!")
        final_x, final_y = map(int,input("Enter the final x and y position in cm separated by a space: ").split())
        final_x, final_y = world_to_matrix(final_x, final_y, threshold)

    test_plot(matrix_map, threshold, initial_x, initial_y)
    run_A_star(initial_x, initial_y, final_x, final_y, initial_orientation, final_orientation,\
               obstacle_points, obstacle_points_clearance, obstacle_points_config, threshold,\
               entire_region, step_size)
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    runner()
