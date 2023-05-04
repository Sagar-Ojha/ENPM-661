"""
Description: Implementation of RRT*-Smart algorithm on a point robot
Date: 5/09/2023
Author: Sagar Ojha (as03050@umd.edu), Fnu Obaid Ur Rahman (obdurhmn@umd.edu)
"""
import heapq as hq
import numpy as np
import time
import cv2
import matplotlib.pyplot as plt

#--------------------------------------------------------------------------------------------------
def animate_RRT_star_Smart(entire_region, t):
    """! Plots the coordinates that have been explored and the optimal path to the goal
    @param entire_region The array of coordinates of the obstacle region
    @param t Threshold for the matrix map
    @return None
    """
    x_obs = []
    y_obs = []
    x_outer = []
    y_outer = []
    for x in range(len(entire_region)):
        for y in range(len(entire_region[x])):
            if entire_region[x][y] == -1:
                x_obs.append(x)
                y_obs.append(y)
            elif entire_region[x][y] == -2:
                x_outer.append(x)
                y_outer.append(y)

    plt.xlim([0,int(100/t)])
    plt.ylim([0,int(100/t)])
    plt.scatter(x_obs, y_obs, marker = "s", s = 0.35, c = 'red')
    plt.scatter(x_outer, y_outer, marker = "s", s = 0.35, c = 'black')
    plt.show()
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def configuration_space(t):
    """! Creates a matrix map of the configuration space
    @return A matrix implementing the configuration map
    """
    entire_region = np.zeros([int(101/t),int(101/t)])

    for x in range(len(entire_region)):
        for y in range(len(entire_region[x])):
            # Inequalities defining the outer walls
            if ((x*t == 0) or (x*t == 100) or (y*t == 0) or (y*t == 100)):
                entire_region[x][y] = -2    # Just for the outer walls
            # Inequalities defining the long verticle rectangular obstacle region
            elif ((x*t <= 83) and (x*t >= 73) and (y*t <= 96) and (y*t >= 16)):
                entire_region[x][y] = -1
            # Inequalities defining the long horizontal rectangular obstacle region
            elif ((x*t <= 73) and (x*t >= 13) and (y*t <= 62) and (y*t >= 56)):
                entire_region[x][y] -= 1
            # Inequalities defining the long horizontal rectangular obstacle region
            elif ((x*t <= 95) and (x*t >= 83) and (y*t <= 46) and (y*t >= 40)):
                entire_region[x][y] -= 1

    return entire_region
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def runner():
    threshold = 0.5 # matrix-resolution threshold
    entire_region = configuration_space(threshold)
    animate_RRT_star_Smart(entire_region, threshold)
    return
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    runner()