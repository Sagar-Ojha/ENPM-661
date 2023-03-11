"""
Description: Solution to 8 puzzle problem using BFS algorithm
Date: 2/24/2023
Author: Sagar Ojha (as03050@umd.edu)
Future work: Needs an inversion count function to check if the puzzle is solvable or not.
             Needs to break down new_nodes() into 2 functions for better modularity.
"""
from copy import deepcopy

#----------------------------------------------------------------------------------------
def generate_path(node_path, visited_list):
    """! Back tracking algorithm that provides nodes to reach the final state.
    @param node_path Array that contains the final node.
    @param visited_list List of the visited nodes.
    @return None: Just updates the node_path array with the parent nodes sequentially.
    """
    parent_index = node_path[-1][2]
    while parent_index != -1:
        node_path.append(visited_list[parent_index])
        parent_index = node_path[-1][2]
#----------------------------------------------------------------------------------------

#----------------------------------------------------------------------------------------
def new_nodes(actions, all_node_states, unvisited_list, blank_location, node_state,\
               node_index, final_state):
    """! Creates new nodes based on actions, blank_location, and node_state. And updates 
    them to the unvisited_list.
    @param actions The feasible action set for the given blank_location
    @param all_node_states Contains all the states that have been created so far
    @param unvisited_list Contains the nodes that are yet to be explored
    @param blank_location The index of the blank tile
    @param node_state The state of the 8-puzzle problem
    @param node_index The index of the parent node
    @param final_state The final state of the 8-puzzle problem
    @return "yes" if new_node_state is final_state and "no" otherwise.
    """
    if ('up' in actions):
        new_node_state = blank_tile_up(blank_location, node_state)
        if (new_node_state not in all_node_states):
            new_node_index = len(all_node_states)
            all_node_states.append(new_node_state)
            new_node = [new_node_state, new_node_index, node_index]
            unvisited_list.append(new_node)
            if (new_node_state == final_state): return "yes"
    if ('down' in actions):
        new_node_state = blank_tile_down(blank_location, node_state)
        if (new_node_state not in all_node_states):
            new_node_index = len(all_node_states)
            all_node_states.append(new_node_state)
            new_node = [new_node_state, new_node_index, node_index]
            unvisited_list.append(new_node)
            if (new_node_state == final_state): return "yes"
    if ('left' in actions):
        new_node_state = blank_tile_left(blank_location, node_state)
        if (new_node_state not in all_node_states):
            new_node_index = len(all_node_states)
            all_node_states.append(new_node_state)
            new_node = [new_node_state, new_node_index, node_index]
            unvisited_list.append(new_node)
            if (new_node_state == final_state): return "yes"
    if ('right' in actions):
        new_node_state = blank_tile_right(blank_location, node_state)
        if (new_node_state not in all_node_states):
            new_node_index = len(all_node_states)
            all_node_states.append(new_node_state)
            new_node = [new_node_state, new_node_index, node_index]
            unvisited_list.append(new_node)
            if (new_node_state == final_state): return "yes"
    return "no"
#----------------------------------------------------------------------------------------

#----------------------------------------------------------------------------------------
def blank_tile_up(blank_location, node_state):
    """! Create a new node after moving the blank tile up in the given node_state
    @param blank_location The index of the blank tile in node_state array
    @param node_state The state of the node represented as an array
    @return The new state after moving the blank tile 1 step up
    """
    new_node_state = deepcopy(node_state)
    new_node_state[blank_location] = new_node_state[blank_location - 1]
    new_node_state[blank_location - 1] = 0
    return new_node_state
#----------------------------------------------------------------------------------------

#----------------------------------------------------------------------------------------
def blank_tile_down(blank_location, node_state):
    """! Create a new node after moving the blank tile down in the given node_state
    @param blank_location The index of the blank tile in node_state array
    @param node_state The state of the node represented as an array
    @return The new state after moving the blank tile 1 step down
    """
    new_node_state = deepcopy(node_state)
    new_node_state[blank_location] = new_node_state[blank_location + 1]
    new_node_state[blank_location + 1] = 0
    return new_node_state
#----------------------------------------------------------------------------------------

#----------------------------------------------------------------------------------------
def blank_tile_left(blank_location, node_state):
    """! Create a new node after moving the blank tile left in the given node_state
    @param blank_location The index of the blank tile in node_state array
    @param node_state The state of the node represented as an array
    @return The new state after moving the blank tile 1 step left
    """
    new_node_state = deepcopy(node_state)
    new_node_state[blank_location] = new_node_state[blank_location - 3]
    new_node_state[blank_location - 3] = 0
    return new_node_state
#----------------------------------------------------------------------------------------

#----------------------------------------------------------------------------------------
def blank_tile_right(blank_location, node_state):
    """! Create a new node after moving the blank tile right in the given node_state
    @param blank_location The index of the blank tile in node_state array
    @param node_state The state of the node represented as an array
    @return The new state after moving the blank tile 1 step right
    """
    new_node_state = deepcopy(node_state)
    new_node_state[blank_location] = new_node_state[blank_location + 3]
    new_node_state[blank_location + 3] = 0
    return new_node_state
#----------------------------------------------------------------------------------------

#----------------------------------------------------------------------------------------
def actions_allowed(blank_location):
    """! Checks the allowable actions in the 8-puzzle problem
    @param blank_location The index of the blank tile in the node_state array
    @return An array that contains the allowable actions for the given blank_location
    """
    actions = ['left', 'right', 'up', 'down']
    bl = blank_location
    if ((bl == 0) or (bl == 3) or (bl == 6)): actions.remove('up')
    if ((bl == 0) or (bl == 1) or (bl == 2)): actions.remove('left')
    if ((bl == 2) or (bl == 5) or (bl == 8)): actions.remove('down')
    if ((bl == 6) or (bl == 7) or (bl == 8)): actions.remove('right')
    return actions
#----------------------------------------------------------------------------------------

#----------------------------------------------------------------------------------------
def run_8_puzzle_BFS(initial_state, final_state):
    """! Runs BFS algorithm based on the flow-chart in the instructions document.
    @param initial_state The initial state of the puzzle
    @param final_state The final state of the puzzle
    @return None
    """
    if initial_state == final_state:
        print(f'The initial state is the final state.')
        return
    node_state = initial_state
    node_index = 0
    node_parent_index = -1

    # The state of the node (node_state) is stored as an array in this implementation
    # Each node has its state, index, and the parent index
    node = [node_state, node_index, node_parent_index]
    # unvisited_list stores the node that is to be explored
    unvisited_list = [node]
    # visited_list stores the nodes that are explored
    visited_list = []
    # all_node_states stores all the node states that are created
    all_node_states = [node_state]
    # final_found is the flag to check if the final_state is obtained or not
    if (node_state == final_state): final_found = "yes"
    else: final_found = "no"

    # BFS algorithm
    while ((unvisited_list != []) and (final_found == "no")):
        node_state = unvisited_list[0][0]
        node_index = unvisited_list[0][1]
        
        # Put the node_state from the unvisited_list to the visited_list
        visited_list.append(unvisited_list.pop(0))

        # Obtain the location of the blank tile in the node_state
        blank_location = node_state.index(0)
        # Check what actions are allowed based off of the location of the blank tile
        actions = actions_allowed(blank_location)

        # Create a new_node based on the actions and append them to unvisited_list
        final_found = new_nodes(actions, all_node_states, unvisited_list, blank_location,\
                                 node_state, node_index, final_state)

    node_path = [unvisited_list[-1]]
    generate_path(node_path, visited_list)
    
    node_path.reverse()
    # It's reversed so that node_path[0] is initial_state & node_path[-1] is final_state

    if (final_found == "no"): print("Solution Not Found")
    else:
        file_nodePath = open('nodePath.txt', 'w')
        for i in node_path:
            for j in i[0]:
                file_nodePath.write(str(j) + ' ')
            file_nodePath.write('\n')
        file_nodePath.close()

        file_Nodes = open('Nodes.txt', 'w')
        for i in visited_list:
            for j in i[0]:
                file_Nodes.write(str(j) + ' ')
            file_Nodes.write('\n')
        file_Nodes.close()

        file_NodesInfo = open('NodesInfo.txt', 'w')
        # NodesInfo contains information about the explored nodes only
        # Column 1: Node Index, Column 2: Node Parent Index, Column 3: Node State
        for i in visited_list:
            file_NodesInfo.write(str(i[1]) + '  ' + str(i[2]) + '  ' + str(i[0]) + '\n')
        file_NodesInfo.close()
#----------------------------------------------------------------------------------------

#----------------------------------------------------------------------------------------
if __name__ == "__main__":
    '''
    Change the initial and final states of the puzzle following the convention below:
    | a | d | g |
    | b | e | h |  ==> represented in the array form as [a, b, c, d, e, f, g, h, i]
    | c | f | i |
    '''
    initial_state = [1, 4, 8, 2, 0, 6, 3, 5, 7]
    final_state = [1, 4, 7, 2, 5, 8, 3, 6, 0]
    run_8_puzzle_BFS(initial_state, final_state)