"""
Description: Implementation of RRT*-Smart algorithm on a point robot
Date: 5/09/2023
Author: Sagar Ojha (as03050@umd.edu), Fnu Obaid Ur Rahman (obdurhmn@umd.edu)
"""
import numpy as np
import time
import cv2
import copy
import matplotlib.pyplot as plt

#--------------------------------------------------------------------------------------------------
def animate_RRT_star_Smart(entire_region, t, optimal_path, z_beacons):
    """! Plots the coordinates that have been explored and the optimal path to the goal
    @param entire_region The array of coordinates of the obstacle region
    @param t Threshold for the matrix map
    @param optimal_path The optimal_path given by RRT*
    @param z_beacons The optimal path givn by RRT*-Smart
    @return None
    """
    x_obs = []
    y_obs = []
    x_outer = []
    y_outer = []
    x_exp = [] # Explored nodes
    y_exp = []
    x_opt = []
    y_opt = []
    x_beacon = []
    y_beacon = []
    
    for x in range(len(entire_region)):
        for y in range(len(entire_region[x])):
            if entire_region[x][y][2] == -1:
                x_obs.append(x*t)
                y_obs.append(y*t)
            elif entire_region[x][y][2] == -2:
                x_outer.append(x*t)
                y_outer.append(y*t)
            elif (entire_region[x][y][2] > 1):
                x_exp = [x*t, entire_region[x][y][0]]
                y_exp = [y*t, entire_region[x][y][1]]
                plt.plot(x_exp, y_exp, linewidth = '0.5', color = 'magenta')

    if (len(optimal_path) != 0):
        for node in optimal_path:
            x_opt.append(node[0])
            y_opt.append(node[1])
        plt.plot(x_opt, y_opt, linewidth = '1', color = 'black')
    
    for beacon in z_beacons:
        print(beacon)
        x_beacon.append(beacon[0])
        y_beacon.append(beacon[1])
    plt.plot(x_beacon, y_beacon, linewidth = '1', color = 'green')

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
def nodes_visible(entire_region, t, parent, child):
    """! Checks if there is any obstacle between parent and child nodes/beacons """
    step_size = 1
    angle = np.arctan2((child[1] - parent[1]), (child[0] - parent[0]))
    full_length = eucledian_distance(parent[0], parent[1], child[0], child[1])
    intermediate_length = 0
    while (full_length > intermediate_length):
        x = parent[0] + step_size * np.cos(angle)
        y = parent[1] + step_size * np.sin(angle)
        step_size += 1
        mat_x, mat_y = matrix_indices([x, y], t)
        if entire_region[mat_x][mat_y][2] < 0: return False
        intermediate_length += 1

    return True
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def path_optimization(entire_region, t, RRT_star_optimal_path):
    """! Returns the array that has successive beacon nodes and the direct cost """
    Z_beacons = [RRT_star_optimal_path[0]]
    # mat_x_final, mat_y_final = matrix_indices(RRT_star_optimal_path[-1], t)
    # direct_cost = entire_region[mat_x_final][mat_y_final][2]

    while ((Z_beacons[-1][0] != RRT_star_optimal_path[-1][0]) and\
            (Z_beacons[-1][1] != RRT_star_optimal_path[-1][1])):
        # print(Z_beacons[-1])
        # print(RRT_star_optimal_path)
        start_index = 0
        for i in range(len(RRT_star_optimal_path)):
            if ((RRT_star_optimal_path[i][0] == Z_beacons[-1][0]) and\
                (RRT_star_optimal_path[i][1] == Z_beacons[-1][1])):
                start_index = i
        for i in range(len(RRT_star_optimal_path), start_index, -1):
            parent = Z_beacons[-1]
            child = RRT_star_optimal_path[i-1]
            # print(start_index, i)
            if (i == start_index + 1):
                Z_beacons.append(RRT_star_optimal_path[i])
                break
            if nodes_visible(entire_region, t, parent, child):
                Z_beacons.append(child)
                break
    
    direct_cost = 0
    for i in range(len(Z_beacons)):
        if (i == (len(Z_beacons) - 1)):
            break
        direct_cost += eucledian_distance(Z_beacons[i][0], Z_beacons[i][1],\
             Z_beacons[i+1][0], Z_beacons[i+1][1])

    return Z_beacons, direct_cost
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def optimal_path_RRT_star(entire_region, t, z_new):
    """! Backtracks the optimal path for RRT* algorithm"""
    optimal_path = [z_new]
    cost = 0
    while cost != 0.1: # Cost of z_init is 1
        mat_x, mat_y = matrix_indices(z_new, t)
        z_new = entire_region[mat_x][mat_y][0:2]
        cost = entire_region[mat_x][mat_y][2]
        optimal_path.insert(0, z_new)
    optimal_path.pop(0)
    return optimal_path
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def initial_path_found(z_goal, z_new):
    """! Returns True is initial distance is found """
    error = eucledian_distance(z_goal[0], z_goal[1], z_new[0], z_new[1])
    if error < 5:
        return True
    else: return False
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def rewire(entire_region, t , z_near, z_new, z_min):
    """! Rewires z_new as the new parent to nodes in z_near if doing so decreases the cost-to-come
    for the nodes in z_near"""
    mat_x_parent, mat_y_parent = matrix_indices(z_new, t)
    parent_cost = entire_region[mat_x_parent][mat_y_parent][2]
    for nodes in z_near:
        # if ((nodes[0] != z_min[0]) and (nodes[1] != z_min[1])): # Done to make sure tree is not cut
        mat_x, mat_y = matrix_indices(nodes, t)
        distance_from_z_new = eucledian_distance(z_new[0], z_new[1], nodes[0], nodes[1])
        new_cost = parent_cost + distance_from_z_new
        if ((new_cost < entire_region[mat_x][mat_y][2]) and \
            (nodes_visible(entire_region, t, z_new, nodes))):
            entire_region[mat_x][mat_y] = np.array([z_new[0], z_new[1], new_cost])
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def chosen_parent(z_near, z_nearest, z_new, entire_region, t):
    """! Returns the lowest cost-to-come node as the parent for z_new """
    parent_node = z_nearest
    mat_x_parent, mat_y_parent = matrix_indices(z_nearest, t)
    cost_to_come = entire_region[mat_x_parent][mat_y_parent][2] +\
                            eucledian_distance(z_nearest[0], z_nearest[1], z_new[0], z_new[1])
    for possible_parent in z_near:
        mat_x_parent, mat_y_parent = matrix_indices(possible_parent, t)
        cost_to_come_current = entire_region[mat_x_parent][mat_y_parent][2] +\
          eucledian_distance(possible_parent[0], possible_parent[1], z_new[0], z_new[1])
        if (cost_to_come_current < cost_to_come):
            parent_node = possible_parent
            cost_to_come = cost_to_come_current

    return parent_node, cost_to_come
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def near(entire_region, t, z_new, search_radius):
    """! Returns an array of the nodes that fall within the search_radius of z_new node"""
    z_near = []
    for i in range(z_new[0] - search_radius, z_new[0] + (search_radius + 1)):
        if ((i > 0) and (i < 100)):
            for j in range(z_new[1] - search_radius, z_new[1] + (search_radius + 1)):
                if ((j > 0) and (j < 100)):
                    # if ((i ** 2 + j ** 2) < (search_radius ** 2)):
                    node = np.array([i, j])
                    mat_x, mat_y = matrix_indices(node, t)
                    if ((entire_region[mat_x][mat_y][2] > 0) and (i != z_new[0])\
                        and (j != z_new[1])):
                        z_near.append(node)
    return z_near
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def obstacle_free(coordinate, entire_region, t):
    """! Checks if z_new is in obstacle region or not """
    mat_x, mat_y = matrix_indices(coordinate, t)
    if entire_region[mat_x][mat_y][2] < 0:
        return False
    else: return True
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def eucledian_distance(x1, y1, x2, y2):
    """! Calculates the distance between two points """
    distance = ((x2 - x1)**2 + (y2 - y1)**2)**0.5
    return distance
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def steer(z_nearest, z_rand):
    """! Finds the new node between z_nearest and z_rand after performing action on z_nearest """
    action_step_size = 5
    if (eucledian_distance(z_nearest[0], z_nearest[1], z_rand[0], z_rand[1]) > 5):
        # Angle between the horizontal and the vector connecting z_nearest to z_rand
        angle_with_horizontal = np.arctan2((z_rand[1] - z_nearest[1]), (z_rand[0] - z_nearest[0]))
        x_new = z_nearest[0] + action_step_size * np.cos(angle_with_horizontal)
        y_new = z_nearest[1] + action_step_size * np.sin(angle_with_horizontal)
        z_new = np.array([int(x_new), int(y_new)])
    else:
        z_new = z_rand
    return z_new
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def nearest(entire_region, z_rand, t):
    """! Searches around the boundary of a square for presence of a node and returns that node
    Ideally, we would want to search around a circle rather than a square but that has its own caveats
    like missing a crucial node because of truncation and round off error while formulating circle"""
    nearest_node = []
    radius = 1
    while len(nearest_node) == 0:
        for i in range(z_rand[0] - radius, z_rand[0] + (radius + 1)):
            if ((i > 0) and (i < 100)):
                if ((z_rand[1] - radius) > 0):
                    node_bottom = np.array([i, z_rand[1] - radius])
                    mat_x, mat_y = matrix_indices(node_bottom, t)
                    if (entire_region[mat_x][mat_y][2] > 0):
                        nearest_node.append(node_bottom)
                if ((z_rand[1] + radius) < 100):
                    node_top = np.array([i, z_rand[1] + radius])
                    mat_x, mat_y = matrix_indices(node_top, t)
                    if (entire_region[mat_x][mat_y][2] > 0):
                        nearest_node.append(node_top)

        for j in range(z_rand[1] - radius + 1, z_rand[1] + radius):
            if ((j > 0) and (j < 100)):
                if ((z_rand[0] - radius) > 0):
                    node_left = np.array([z_rand[0] - radius, j])
                    mat_x, mat_y = matrix_indices(node_left, t)
                    if (entire_region[mat_x][mat_y][2] > 0):
                        nearest_node.append(node_left)
                if ((z_rand[0] + radius) < 100):
                    node_right = np.array([z_rand[0] + radius, j])
                    mat_x, mat_y = matrix_indices(node_right, t)
                    if (entire_region[mat_x][mat_y][2] > 0):
                        nearest_node.append(node_right)
        radius += 1

    return nearest_node[0]
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def random_sampling(entire_region, t):
    """! Returns a random coordinate within 1 to 99 """
    x, y = np.random.randint(1, 99, 2)
    mat_x, mat_y = matrix_indices([x, y], t)
    while (entire_region[mat_x][mat_y]).any() != 0:
        x, y = np.random.randint(1, 99, 2)
        mat_x, mat_y = matrix_indices([x, y], t)
    
    return x, y
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def intelligent_sampling(z_beacons, entire_region, t):
    """! Returns a random coordinate near z_beacon within biasing_radius"""
    biasing_radius = 5 # Not exactly gives values within a circle but more like within a square
    if (len(z_beacons) == 2): i = 1
    else: i = np.random.randint(1, len(z_beacons) - 2)
    x = np.random.randint(z_beacons[i][0] - biasing_radius, z_beacons[i][0] + biasing_radius)
    y = np.random.randint(z_beacons[i][1] - biasing_radius, z_beacons[i][1] + biasing_radius)
    if ((x > 100) or (x < 0)): x = 60
    if ((y > 100) or (y < 0)): y = 50
    mat_x, mat_y = matrix_indices([x, y], t)
    while (entire_region[mat_x][mat_y][2] < 0):
        x = np.random.randint(z_beacons[i][0] - biasing_radius, z_beacons[i][0] + biasing_radius)
        y = np.random.randint(z_beacons[i][1] - biasing_radius, z_beacons[i][1] + biasing_radius)
        if ((x > 100) or (x < 0)): x = 60
        if ((y > 100) or (y < 0)): y = 50
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
    num_iteration = 5000 # Iteration number for the search
    biasing_ratio = 2 # Could implement a dynamic biasing ratio (Eqn (1) in Jauwaria article)
    iter_at_soln = num_iteration # Iteration number when the initial path is found
    # By default iter_at_soln is set to num_iteration. It'll change once initial path is found

    z_init = np.array([50, 50]) # Start node
    z_goal = np.array([95, 85]) # Goal node
    path_found = "no"
    z_beacons = [z_init, z_goal] # Initializing the first value for z_beacon
    direct_cost_old = 0.1 # Initializing the default value for direct_cost_old
    direct_cost_new = 0.2

    parent_node = np.array([-1, -1]) # Parent state for the first node is set as [-1, -1]
    mat_x, mat_y = matrix_indices(z_init, t) # Get the indices in the matrix map

    # Parent-node and the cost-to-come values are set stored in the 3rd dimension of the matrix map
    # That way we don't need to maintain another data structure, specifically tree
    entire_region[mat_x][mat_y] = np.array([parent_node[0], parent_node[1], 0.1])
    # Cost-to-come for the first node is set to 0.1 by default    
    RRT_star_optimal_path = []

    for i in range(1, num_iteration+1):
        if ((iter_at_soln != num_iteration) and (((i - iter_at_soln) % biasing_ratio) == 0)):
            z_rand = intelligent_sampling(z_beacons, entire_region, t)
        else:
            z_rand = random_sampling(entire_region, t)
        # z_rand = random_sampling(entire_region, t)
        
        # Getting the nearest node to z_rand
        z_nearest = nearest(entire_region, z_rand, t)
        # print(z_nearest)

        # Setting up the new node z_new after performing the action on z_nearest
        z_new = steer(z_nearest, z_rand) # Could get action command as well

        # Check if z_new is in the obstacle region
        if (obstacle_free(z_new, entire_region, t) == True):
            search_radius = 5 # Ideally, action_step_size has to be the same as well
            z_near = near(entire_region, t, z_new, search_radius)
            z_min, cost_to_come = chosen_parent(z_near, z_nearest, z_new, entire_region, t)

            mat_x_new, mat_y_new = matrix_indices(z_new, t)
            if (nodes_visible(entire_region, t, z_min, z_new)):
                entire_region[mat_x_new][mat_y_new] = np.array([z_min[0], z_min[1], cost_to_come])
            # We store the physical coordinates of the parent node rather than the parent's matrix indices
            # Above steps are equivalent to insertnode function in the algorithm
            # Upto here is RRT----------------------------------------------------------------
                rewire(entire_region, t, z_near, z_new, z_min)
            # Upto here is RRT*---------------------------------------------------------------

            if (initial_path_found(z_goal, z_new) and (iter_at_soln == num_iteration)):
                iter_at_soln = i
                z_goal = z_new # Done just to repeat the same simulation, thus easier to debug
                path_found = "yes"
                direct_cost_old = cost_to_come
                print(f'Goal: {z_goal}')
                print(f'Iteration when initial path found: {iter_at_soln}')
                print(f'RRT* cost: {cost_to_come}')
                RRT_star_optimal_path = optimal_path_RRT_star(entire_region, t, z_goal)
                # Cost is compared in optimal_path_RRT_star funciton. So, no need to pass z_init
                z_beacons, direct_cost_new = path_optimization(entire_region, t, RRT_star_optimal_path)
            
        if (path_found == "yes"):
            RRT_star_optimal_path = optimal_path_RRT_star(entire_region, t, z_goal)
            # Cost is compared in optimal_path_RRT_star funciton. So, no need to pass z_init
            Z_beacons, direct_cost_new = path_optimization(entire_region, t, RRT_star_optimal_path)

        if direct_cost_new < direct_cost_old:
            direct_cost_old = direct_cost_new
            z_beacons = copy.deepcopy(Z_beacons)
            # Necessary to deep copy otherwise the if condition is not effective

    RRT_star_Smart_cost = 0
    for i in range(len(z_beacons)):
        if (i == (len(z_beacons) - 1)):
            break
        RRT_star_Smart_cost += eucledian_distance(z_beacons[i][0], z_beacons[i][1],\
             z_beacons[i+1][0], z_beacons[i+1][1])
    print(f'RRT*-Smart cost: {RRT_star_Smart_cost}')

    animate_RRT_star_Smart(entire_region, t, RRT_star_optimal_path, z_beacons)
    return
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
def configuration_space(t):
    """! Creates a matrix map of the configuration space
    @return A matrix implementing the configuration map
    """
    entire_region = np.zeros([int(101/t),int(101/t), 3])

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
    np.random.seed(10) # Seed for random sampling
    RRT_star_smart(entire_region, threshold)
    return
#--------------------------------------------------------------------------------------------------

#--------------------------------------------------------------------------------------------------
if __name__ == "__main__":
    runner()