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
    plt.scatter(x_obs, y_obs, marker = "s", c = 'red')
    plt.scatter(x_outer, y_outer, marker = "s", c = 'black')
    plt.show()
#--------------------------------------------------------------------------------------------------

def animate_sample_point(visited_nodes, entire_region):

    canvas = np.zeros((203,203, 3), dtype=np.uint8)
    display_canvas_anim = canvas.copy()
    scale_percent = 600  # percent of original size
    width = int(display_canvas_anim.shape[1] * scale_percent / 100)
    height = int(display_canvas_anim.shape[0] * scale_percent / 100)
    canvas = cv2.resize(display_canvas_anim, (width, height), interpolation=cv2.INTER_LINEAR)
    
    for x in range(len(entire_region)):
        for y in range(len(entire_region[x])):
            if entire_region[x][y] == -1:
                # Draw a red circle at the obstacle location
                cv2.circle(canvas, (int(x * scale_percent / 100), int(height - (y* scale_percent / 100))), 4, (0, 0, 255), -1)
            elif entire_region[x][y] == -2:
                # Draw a black circle at the outer boundary location
                cv2.circle(canvas, (int(x * scale_percent / 100), int(height - (y* scale_percent / 100))), 4, (255, 0, 255), -1)

    for i in range(len(visited_nodes)):
        x = visited_nodes[i][0]
        y = visited_nodes[i][1]
        cv2.circle(canvas, (int(x * scale_percent / 100), int(height - (y* scale_percent / 100))), radius=4, color=(255, 0, 0), thickness=-1)
        # Display the canvas
        cv2.imshow('Canvas', canvas)
        cv2.waitKey(200)
        #time.sleep(0.1)

    cv2.imshow('Canvas', canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#--------------------------------------------------------------------------------------------------
visited_nodes = []
def sample_point(iterations, threshold, entire_region):
    global visited_nodes
    
    t = threshold
    iter = iterations

    for i in range(iter):
        # Get a random coordinate
        x, y = np.random.randint(0, 100, size=2)
        # Check if the coordinate has already been used

        new_x = x / t
        new_y = y / t

        if entire_region[int(new_x)][int(new_y)] == -1 or entire_region[int(new_x)][int(new_y)] == -2:
            continue # Checks if it is in obstacle space

        if (new_x, new_y) not in visited_nodes:# Check if the coordinate is outside the obstacles
            visited_nodes.append((new_x,new_y))
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
    iterations = 100
    animate_RRT_star_Smart(entire_region, threshold)
    sample_point(iterations, threshold, entire_region)
    animate_sample_point(visited_nodes, entire_region)
    return
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    runner()