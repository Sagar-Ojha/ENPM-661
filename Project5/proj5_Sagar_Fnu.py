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
                x_obs.append(x*t)
                y_obs.append(y*t)
            elif entire_region[x][y] == -2:
                x_outer.append(x*t)
                y_outer.append(y*t)

    plt.xlim([0,int(100)])
    plt.ylim([0,int(100)])
    plt.scatter(x_obs, y_obs, marker = "s", s = 0.35, c = 'red')
    plt.scatter(x_outer, y_outer, marker = "s", s = 0.35, c = 'black')
    plt.show()
#--------------------------------------------------------------------------------------------------

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

#--------------------------------------------------------------------------------------------------
def random_sampling(entire_region, t):
    """! Returns a random coordinate within 1 to 99 """
    x, y = np.random.randint(1, 99, 2)
    mat_x, mat_y = matrix_indices([x, y], t)
    while entire_region[mat_x][mat_y] != 0:
        x, y = np.random.randint(1, 99, 2)
        mat_x, mat_y = matrix_indices([x, y], t)
    
    return x, y
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def intelligent_sampling(z_beacon, entire_region, t):
    """! Returns a random coordinate near z_beacon within biasing_radius"""
    biasing_radius = 3 # Not exactly gives values within a circle but more like within a square
    x = np.random.randint(z_beacon[0] - biasing_radius, z_beacon[0] + biasing_radius)
    y = np.random.randint(z_beacon[1] - biasing_radius, z_beacon[1] + biasing_radius)
    mat_x, mat_y = matrix_indices([x, y], t)
    while entire_region[mat_x][mat_y] < 0:
        x = np.random.randint(z_beacon[0] - biasing_radius, z_beacon[0] + biasing_radius)
        y = np.random.randint(z_beacon[1] - biasing_radius, z_beacon[1] + biasing_radius)
        mat_x, mat_y = matrix_indices([x, y], t)
    
    return np.array([x, y])
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def matrix_indices(physical_coordinates, t):
    """! Returns the matrix indices corresponding to the physical coordinates """
    return int(physical_coordinates[0]/t), int(physical_coordinates[1]/t)
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def RRT_star_smart(entire_region, t):
    """! Implements RRT*-SMART algorithm
    @param entire_region Matrix implementation of the configuration space
    @param t Threshold for the matrix map
    @return None
    """
    num_iteration = 1000 # Iteration number for the search
    biasing_ratio = 5 # Could implement a dynamic biasing ratio (Eqn (1) in Jauwaria article)
    iter_at_soln = num_iteration # Iteration number when the initial path is found
    # By default iter_at_soln is set to num_iteration. It'll change once initial path is found

    z_init = np.array([50, 50]) # Start node
    z_goal = np.array([95, 85]) # Goal node
    z_beacon = z_init # Initializing the first value for z_beacon

    node = [z_init, np.array([-1, -1])] # Each node consists of the state and its parent
    # Parent state for the first node is set as [-1, -1]
    mat_x, mat_y = matrix_indices(z_init, t) # Get the indices in the matrix map
    entire_region[mat_x][mat_y] = 1 # Cost-to-come values are set to the nodes that are stored in T
    # Cost-to-come value for z_init is set to 1 (just for an easier implementation)
    
    T = [node]
    # print(z_init, mat_x, mat_y)

    for i in range(1, num_iteration+1):
        if ((iter_at_soln != num_iteration) and (((i - iter_at_soln) % biasing_ratio) == 0)):
            z_rand = intelligent_sampling(z_beacon, entire_region)
        else:
            z_rand = random_sampling(entire_region)


    return
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
    threshold = 0.25 # matrix-resolution threshold
    entire_region = configuration_space(threshold)
    np.random.seed(1) # Seed for random sampling
    RRT_star_smart(entire_region, threshold)
    # animate_RRT_star_Smart(entire_region, threshold)
    return
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    runner()