import numpy as np
import matplotlib.pyplot as plt
from math import sqrt


class Node:
    """
        A node class for A* Pathfinding
        parent is parent of the current Node
        position is current position of the Node in the maze
        g is cost from start to current Node
        h is heuristic based estimated cost for current Node to end Node
        f is total cost of present node i.e. :  f = g + h
    """

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        self.g = 0
        self.h = 0
        self.f = 0
    def __eq__(self, other):
        return self.position == other.position

#This function return the path of the search
def return_path(current_node,maze):
    path = []
    no_rows, no_columns = np.shape(maze)
    # here we create the initialized result maze with -1 in every position
    result = [[-1 for i in range(no_columns)] for j in range(no_rows)]
    current = current_node
    while current is not None:
        path.append(current.position)
        current = current.parent
    # Return reversed path as we need to show from start to end path
    path = path[::-1]
    start_value = 0
    # we update the path of start to end found by A-star serch with every step incremented by 1
    for i in range(len(path)):
        result[path[i][0]][path[i][1]] = start_value
        start_value += 1

    return path

# Default search on a grid maze (implementation of Lab4)
def search(maze, start, end, scale_factor):
    """
        Returns a list of tuples as a path from the given start to the given end in the given maze
        :param maze: the costMap
        :param start: starting position as cell positions
        :param end: goal position as cell positions
        :pram scale_factor: scaling factor to reduce the maze and search to speed up the search
        :return: path as tuples from the given start to the given end
    """

    maze = maze.copy().T
    maze = maze[::scale_factor, ::scale_factor]  

    # Create start and end node with initized values for g, h and f
    start_node = Node(None, tuple(start))
    start_node.g = start_node.h = start_node.f = 0

    end_node = Node(None, tuple(end))
    end_node.g = end_node.h = end_node.f = 0

    # Initialize both yet_to_visit and visited list
    # in this list we will put all node that are yet_to_visit for exploration. 
    # From here we will find the lowest cost node to expand next
    yet_to_visit_dict = {} # will save the node, key is the position (tuple)
    # # in this list we will put all node those already explored so that we don't explore it again    
    visited_dict = {}      # only save the True values, key is the position (tuple)

    # Add the start node
    yet_to_visit_dict[start_node.position] = start_node
    
    # Adding a stop condition. This is to avoid any infinite loop and stop 
    # execution after some reasonable number of steps
    outer_iterations = 0
    max_iterations = (len(maze) // 2) ** 10

    # what squares do we search . serarch movement is 8 point connectivity

    move  =  [[-1, 0 ], # go up
              [ 0, -1], # go left
              [ 1, 0 ], # go down
              [ 0, 1 ],
              [-1, 1],
              [-1, -1],
              [1, 1],
              [1, -1]] # go right


    """
        1) We first get the current node by comparing all f cost and selecting the lowest cost node for further expansion
        2) Check max iteration reached or not . Set a message and stop execution
        3) Remove the selected node from yet_to_visit list and add this node to visited list
        4) Perofmr Goal test and return the path else perform below steps
        5) For selected node find out all children (use move to find children)
            a) get the current postion for the selected node (this becomes parent node for the children)
            b) check if a valid position exist (boundary will make few nodes invalid)
            c) if any node is a wall then ignore that
            d) add to valid children node list for the selected parent
            
            For all the children node
                a) if child in visited list then ignore it and try next node
                b) calculate child node g, h and f values
                c) if child in yet_to_visit list then ignore it
                d) else move the child to yet_to_visit list
    """
    #find maze has got how many rows and columns 
    no_rows, no_columns = np.shape(maze)
    

    # Loop until you find the end
    
    while len(yet_to_visit_dict) > 0:
        
        # Every time any node is referred from yet_to_visit list, counter of limit operation incremented
        outer_iterations += 1    
        
        # Get the current node
        current_node_position = (-999, -999)
        current_node = Node(None, tuple(current_node_position))
        current_node.f = 999999
        for i_position in yet_to_visit_dict.keys():
            i_node = yet_to_visit_dict[i_position] ## first is g, second is f
            if i_node.f < current_node.f: ## compare the f
                current_node = i_node
                
        # if we hit this point return the path such as it may be no solution or 
        # computation cost is too high
        if outer_iterations > max_iterations:
            print ("giving up on pathfinding too many iterations")
            return return_path(current_node,maze)

        # Pop current node out off yet_to_visit list, add to visited list
        yet_to_visit_dict.pop(current_node.position)
        visited_dict[current_node.position] = True

        # test if goal is reached or not, if yes then return the path
        if current_node == end_node:
            print ("Goal reached")
            return return_path(current_node,maze)

        # Generate children from all adjacent squares
        children = []

        for new_position in move: 

            # Get node position
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Make sure within range (check if within maze boundary)
            if (node_position[0] > (no_rows - 1) or 
                node_position[0] < 0 or 
                node_position[1] > (no_columns -1) or 
                node_position[1] < 0):
                continue

            # Make sure walkable terrain
            if maze[node_position[0],node_position[1]] > 0.8:
                continue

            # Create new node
            new_node = Node(current_node, node_position)

            # Append
            children.append(new_node)

        # Loop through children
        
        for child in children:
  
            # Child is on the visited list (search entire visited list)
            if visited_dict.get(child.position, False):
                continue

            # Create the f, g, and h values
            child.g = current_node.g + sqrt(((child.position[0] - current_node.position[0]) ** 2) + 
                                           ((child.position[1] - current_node.position[1]) ** 2))
            ## Heuristic costs calculated here, this is using eucledian distance
            child.h = sqrt(((child.position[0] - end_node.position[0]) ** 2) + 
                       ((child.position[1] - end_node.position[1]) ** 2)) 

            child.f = child.g + child.h

            # Child is already in the yet_to_visit list and g cost is already lower
            child_node_in_yet_to_visit = yet_to_visit_dict.get(child.position, False)
            if (child_node_in_yet_to_visit is not False) and (child.g >= child_node_in_yet_to_visit.g):
                continue

            # Add the child to the yet_to_visit list
            yet_to_visit_dict[child.position] = child

def dist(p1, p2):
    return sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# [Part 3] TODO Complete the this function so that it is adapted to search a graph provided as PRM instead of a grid maze
# Hint: look at the original A* search above and adapt to the PRM
def search_PRM(points, prm, start, end):
    """
        Returns a list of tuples as a path from the given start to the given end in the given maze
        :param prm: probabilistic roadmap
        :param start: starting position as cell positions (indexes of the costMap)
        :param end: goal position as cell positions (indexes of the costMap)
        :return: path as tuples from the given start to the given end
        :return:
    """
    # Create start and end node with initized values for g, h and f
    start_idx = points.index(tuple(start))
    start_node = Node(None, points[start_idx])
    start_node.g = start_node.h = start_node.f = 0

    end_idx = points.index(tuple(end))
    end_node = Node(None, points[end_idx])
    end_node.g = end_node.h = end_node.f = 0

    path_points = []

    # Adding a stop condition. This is to avoid any infinite loop and stop 
    # execution after some reasonable number of steps
    outer_iterations = 0
    max_iterations = (len(points) // 2) ** 10

    to_visit = {} # dict of index to node, storing nodes to visit
    visited = {} # dict of index to bool, storing visited nodes

    to_visit[start_idx] = start_node # add start node to visited
    pathFound = False # flag indicating if a path is found
    # loop while the nodes to visit isnt empty
    while len(to_visit) > 0:
        # determine node to visit next (node with lowest f score)
        min_f = 999999
        node_to_visit_idx = -1
        for node_idx in to_visit.keys():
            if to_visit[node_idx].f < min_f:
                min_f = to_visit[node_idx].f
                node_to_visit_idx = node_idx
        #extract current node
        curr_node = to_visit[node_to_visit_idx]
        # pop current node from to visit dict
        to_visit.pop(node_to_visit_idx)
        # add current node it visited list
        visited[node_to_visit_idx] = True

        # check if we reached goal
        if node_to_visit_idx == end_idx:
            end_node = curr_node
            pathFound = True
            break

        # if we hit this point return the path such as it may be no solution or 
        # computation cost is too high
        if outer_iterations > max_iterations:
            print ("giving up on pathfinding too many iterations")
            break

        # iterate through neighbours of node we are visiting and potentially update neighbour g, h, and fs
        for n_idx in prm[node_to_visit_idx]:
            #check if not visited - if visited, skip over because its final distance has been finalized
            if visited.get(n_idx, False):
                continue
            # calculate neighbour g, h, and f values with respect to current node
            neighbour = Node(curr_node, points[n_idx])
            neighbour.g = curr_node.g + dist(points[n_idx], curr_node.position)
            neighbour.h = dist(points[n_idx], points[end_idx])
            neighbour.f = neighbour.g + neighbour.h
            # checking if neighbour has been explored (added to to_visit) already (if not we automatically add it to to_visit dict)
            # if it has been explored, we check if the updated g value is greater than the current g value
            # if so, then the previous path it had to it was shorter so we don't update it
            # if the new path (g value) to the neighbour is shorter than the previous path, we update it in to_visit dict
            explored_neighbour = to_visit.get(n_idx, False)
            if (explored_neighbour is not False) and (neighbour.g >= explored_neighbour.g):
                continue
            # updating/adding node to_visit dict
            to_visit[n_idx] = neighbour
        
    if not pathFound:
        print("No path found. Returning empty list")
    else:
        # recursively add points to path_points to form shortest path from start to end
        curr = end_node
        while curr is not None: # when reached start, parent will be None so we stop
            path_points.insert(0, curr.position)
            curr = curr.parent
    
    return path_points
