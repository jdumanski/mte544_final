"""

Probabilistic Road Map (PRM) Planner

This version of the PRM implementation creates a connected graph by creating N_SAMPLES random samples in the free space,
then each sample is connected to max N_KNN neighbors, with a maximum distance of MAX_EDGE_LEN.
The PRM is implemented as follows:
- Parameters that need to be set:
  - N_SAMPLE = number of samples to create in the free space
  - N_KNN = number of neighbors to connect from each sample
  - MAX_EDGE_LEN = maximum distance for creating an edge between two samples
- Sample points are creates with the generate_sample_points function. The samples are randomly created in the free space,
  based on the list of obstacles, and the dimension of the robot. The samples should be at least radius of the robot distance away from any obstacles. 
- Create the roadmap using the generated sample_points, with the generate_road_map function. The roadmap is generated by looping
  over all samples. For each sample, N_KNN edges are created with neighboring samples. In creating the edge, a collision is checked with
  the is_collision function, to create edges that are not overlaying on obstacles. To introduce further control, a MAX_EDGE_LEN is imposed
  so that connected samples (nodes) are not too far from each other.

This code can run standalone without being integrated into the stack with control and navigation: python probabilistic_road_map.py
To do so, set use_map to False. This will help debugging your PRM implementation. It will use the obstacles list, start, and goal 
set in the main function.

Once it is integrated into the stack, set use_map to True and run everything from decisions.py as you have done in the labs.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
from mapUtilities import *
import array

# Parameters of PRM
N_SAMPLE = 1200 # number of sample_points
N_KNN = 20  # number of edge from one sampled point (one node)
MAX_EDGE_LEN = 5 # Maximum edge length, in [m]

show_plot = True

# When set to false, you can run this script stand-alone, it will use the information specified in main
# When set to true, you are expected to use this with the stack and the specified map
use_map = True

def prm_graph(start, goal, obstacles_list, robot_radius, *, rng=None, m_utilities=None):
    """
    Run probabilistic road map graph generation

    :param start: start x,y position
    :param goal: goal x,y position
    :param obstacle_list: obstacles x and y positions
    :param robot_radius: robot radius
    :param rng: (Optional) Random generator
    :pram m_utilities: when using a map, pass the map utilies that contains the costmap information
    :return: roadmap (and list of sample points if using the map)
    """

    print("Generating PRM graph")

    obstacle_kd_tree = KDTree(obstacles_list)

    # By default, we use the set maximum edge length
    max_edge_len = MAX_EDGE_LEN
    # Generate the sample points
    if use_map:
        # [Part 2] TODO The radius of the robot and the maximum edge lengths are given in [m], but the map is given in cell positions.
        # Therefore, when using the map, the radius and edge length need to be adjusted for the resolution of the cell positions
        # Hint: in the map utilities there is the resolution stored

        #divide by resolution since resolution is in m/cell
        #don't need to round since distances between cells can be non integer
        robot_radius = robot_radius/m_utilities.getResolution()
        max_edge_len = max_edge_len/m_utilities.getResolution()

    # Get sample data
    sample_points = generate_sample_points(start, goal,
                                       robot_radius,
                                       obstacles_list,
                                       obstacle_kd_tree, rng)

    # Create the roadmap
    if use_map:
        roadmap = generate_road_map(sample_points, robot_radius, obstacle_kd_tree, max_edge_len, m_utilities)
    else:
        roadmap = generate_road_map(sample_points, robot_radius, obstacle_kd_tree, max_edge_len)

    if show_plot:
        if use_map:
            # When using the map, first convert cells into positions, then plot (for a more intuitive visualization)
            # Plot the sample points
            samples_pos = np.array([m_utilities.cell_2_position([i, j]) for i, j in zip(sample_points[0], sample_points[1])])
            print(samples_pos)
            sample_x = samples_pos[:, 0]
            sample_y = samples_pos[:, 1]
            plt.plot(sample_x, sample_y, ".b")   
            # Plot list of obstacles
            obs_pos = np.array([m_utilities.cell_2_position([i, j]) for i, j in zip(obstacles_list[:, 0], obstacles_list[:, 1])])
            obs_x = obs_pos[:, 0]
            obs_y = obs_pos[:, 1]
            plt.plot(obs_x, obs_y, ".k")  
            # Plot the roadmap
            plot_road_map(roadmap, [sample_x, sample_y])
            # Plot the starting position as a red marker
            s_pos = m_utilities.cell_2_position(start)
            plt.plot(s_pos[0], s_pos[1], "^r")
            # Plot the goal position as a green marker
            g_pos = m_utilities.cell_2_position(goal)
            plt.plot(g_pos[0], g_pos[1], "^g")
        else:
            # plot the sample points
            plt.plot(sample_points[0], sample_points[1], ".b")   
            # Plot list of obstacles
            plt.plot(obstacles_list[:,0], obstacles_list[:,1], ".k")  
            # Plot the roadmap
            plot_road_map(roadmap, sample_points)
            # Plot the starting position as a red marker
            plt.plot(start[0], start[1], "^r")
            # Plot the goal position as a green marker
            plt.plot(goal[0], goal[1], "^g")
        plt.grid(True)
        plt.axis("equal")
        plt.show()
    
    # Return generated roadmap, if using a costmap, return also the list of indices of the sample points
    if use_map:
        sample_points_tuple = list(zip(*sample_points))
        return sample_points_tuple, roadmap
    else:
        return roadmap
    


# Sample points are creates with the generate_sample_points function. The samples are created in the free space,
# based on the list of obstacles, and the dimension of the robot. Two samples should not be closer than the dimension of the robot.
def generate_sample_points(start, goal, rr, obstacles_list, obstacle_kd_tree, rng):
    """
    Generate sample points

    :param start: start x,y position
    :param goal: goal x,y position
    :param obstacle_list: obstacle x and y positions
    :param rr: robot radius
    :param obstacl_kd_tree: KDTree object of obstacles
    :param rng: (Optional) Random number generator
    :return: roadmap
    """
    ox = obstacles_list[:, 0]
    oy = obstacles_list[:, 1]
    sx = start[0]
    sy = start[1]
    gx = goal[0]
    gy = goal[1]

    if rng is None:
        rng = np.random.default_rng()

    # [Part 2] TODO Create list of sample points within the min and max obstacle coordinates
    # Use rng to create random samples. 
    # NOTE: by using rng, the created random samples may not be integers. 
    # When using the map, the samples should be indices of the cells of the costmap, therefore remember to round them to integer values.
    # Hint: you may leverage on the query function of KDTree to find the nearest neighbors

    sample_x, sample_y = [], []

    #determine max x and y values we can sample from using max obstacle x and y values
    max_x = np.max(ox)
    max_y = np.max(oy)
    while len(sample_x) <= N_SAMPLE:
        #rng.random() gives random float between 0 and 1, so multiply value by max_x and max_y to get random coordinate value
        rand_x = rng.random()*max_x
        rand_y = rng.random()*max_y
        #if using map, must round values as they are cell coords
        #if not using map, no need to round
        if use_map:
            rand_x = np.round(rand_x)
            rand_y = np.round(rand_y)
        #calculate distance from sample to closest obstacle
        dist, __ = obstacle_kd_tree.query([rand_x, rand_y], k=1)

        #if distance from sample to closest obstacle is less than robot radius, then sample is invalid, so don't add to sample lists and contintue to next sample
        if dist < rr:
            continue
        #if sample valid, add to sample lists
        sample_x.append(rand_x)
        sample_y.append(rand_y)


    # [Part 2] TODO Add also the start and goal to the samples so that they are connected to the roadmap
    #add start and goal x and y coords to sample lists
    sample_x.append(sx)
    sample_y.append(sy)
    sample_x.append(gx)
    sample_y.append(gy)

    return [sample_x, sample_y]

# line is defined by p1 and p2, and p is point we want to know distance of to line
def distToLine(p, p1, p2):
    x0 = p[0]
    y0 = p[1]
    x1 = p1[0]
    y1 = p1[1]
    x2 = p2[0]
    y2 = p2[1]
    # use distance from point to line formula from 
    # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line 
    num = abs((y2-y1)*x0 - (x2-x1)*y0 + x2*y1 - y2*x1)
    denom = sqrt((y2-y1)**2 + (x2-x1)**2)
    return num/denom


# Check whethere there is a possible collision between two nodes, used for generating the roadmap after sampling the points.
def is_collision(sx, sy, gx, gy, rr, obstacle_kd_tree, max_edge_len):
    """
    Check collisions
    
    :param sx: start node x
    :param sy: start node y
    :param gx: end node x
    :param gy: end node y
    :param rr: robot radius
    :param obstacle_kd_tree: KDTree object of obstacles
    :param max_edge_len: maximum edge length between two nodes

    :return False if no collision, True otherwise

    Checks collision between node sx,sy and gx,gy, accounting for the dimension of the robot rr.
    If max_edge_len is exceeded, also return true as if a collision is verified.
    """

    # [Part 2] TODO Check where there would be a collision with an obstacle between two nodes at sx,sy and gx,gy, and wether the edge between the two nodes is greater than max_edge_len
    # Hint: you may leverage on the query function of KDTree
    
    # check if edge length is too large. If so, return true (i.e. there is a collision, so edge is invalid)
    if sqrt((sx-gx)**2 + (sy-gy)**2) > max_edge_len:
        return True
    
    # check if there would be collision with an obstacle
    # do so by checking if distance from all obstacle points to the line segment formed by points s and g is less than rr.
    # If at least one point is within rr from this line segment (which forms a region where all points in the perimeter are rr from the line segment), 
    # then there is a collision.
    #
    # This region is a stadium (pill shape) with a = ||sg|| and r = rr, whose axis passing through the semi circle ends is coincident with sg,
    # and is centered about the midpoint between s and g. From the generation of sample points, we know that there are no obstacles within rr
    # of s and g. However, this still leaves some area within the rectangle (of length a and width 2r) of the stadium to check. To check if there are 
    # obstacles within this region, we find a region obstacle points between the 2 lines through s and g that are perpendicular to sg (that at least contains all 
    # points within rr of the line segment), and then check if any of them are within
    # rr of the line segment using the distance to a line formula. To implement this, we find all obstacle points within the circle that circrumscribes this rectangle (which)
    # has radius sqrt(r^2 + (a/2)^2)),
    # which is very easy to do with the KD tree by querying it with a maximum search radius. This circle contains all obstacle points within rr of the line segment (since it 
    # circumscribes the reactangle) and points only between the 2 lines mentioned above.
    # This is true because although the circle area goes beyond these lines, the 2 regions that go beyond these lines overlap with the circles centered about s and g with radius rr, 
    # which we know from sample generation don't contain any obstacles.
    #
    # Therefore by calculating the distances of all points within the circle centered at the midpoint between sg with radius sqrt(rr^2 + (||sg||/2)^2), to the line formed 
    # by s and g, we can check if any are within rr, which would indicate there is a collision, which is done below.

    s = np.array([sx, sy])
    g = np.array([gx, gy])
    sg = g-s
    sg_abs = np.linalg.norm(sg)
    # mid point between s and g
    midPoint = s + sg/2
    # upper bound search radius for kd tree to find obstacles
    searchRad = sqrt(rr**2 + (sg_abs/2)**2)
    #kd tree query returns indices of obstacles within searchRad
    __, closeObjectIndices = obstacle_kd_tree.query(midPoint.tolist(), distance_upper_bound = searchRad)
    # if length of data in KD tree is returned, means no obstacles within searchRad exist, so therefore there
    # are no obstacles within rr of the line segment formed by s and g, so edge has no collisions, so we return False
    if closeObjectIndices == len(obstacle_kd_tree.data):
        return False
    # if kd tree returns one value (a scalar), make into list to be compatable with below code
    if not isinstance(closeObjectIndices, np.ndarray):
        closeObjectIndices = [closeObjectIndices]
    # iterate through each obstacle index, and check its distance to the line segment sg
    for i in closeObjectIndices:
        # check if distance from obstacle to line segment sg is less than rr
        # if so, then obstacle is within rr, so return True
        if distToLine(obstacle_kd_tree.data[i], s.tolist(), g.tolist()) < rr:
            return True
        
    return False # No collision

# for a given sample index, finds all valid neighbours it can make edges (connections) with that avoid collisions
def findConnections(sample_kdTree, samples, sample_idx, rr, obstacle_kd_tree, max_edge_len=MAX_EDGE_LEN):
    connections = []
    # get indices of k nearest neighbours
    # we let k = N_KNN+1 since the query will return the sample itself (and we don't connect a sample to itself)
    __, knn_indices = sample_kdTree.query(samples[sample_idx], k=N_KNN+1)
    # if kd tree returns one value (a scalar), make into list to be compatable with below code
    if not isinstance(knn_indices, np.ndarray):
        knn_indices = [knn_indices]
    # iterate through nearest neighbours, and for each one determine if there is a collision
    # if there is no collision, add neighbour index connections list, else continue to next neighbour
    # the collision function also checks if max_edge_len is exceeded, so no need to check in loop
    # skip first index in knn_indices because its index of current sample itself
    for i in range(1, len(knn_indices)):
        neighbour_idx = knn_indices[i]
        if not is_collision(samples[sample_idx][0], samples[sample_idx][1], samples[neighbour_idx][0], samples[neighbour_idx][1], rr, obstacle_kd_tree, max_edge_len):
            # edge is collision free, so add index to connections
            connections.append(neighbour_idx)
    return connections

# Create the roadmap using the generated sample_points, with the generate_road_map function. The roadmap is generated by looping
# over all samples. For each sample, N_KNN edges are created with neighboring samples. In creating the edge, a collision is checked with
# the is_collision function, to create edges that are not overlaying on obstacles. To introduce further control, a MAX_EDGE_LEN is imposed
# so that connected samples (nodes) are not too far from each other.
def generate_road_map(sample_points, rr, obstacle_kd_tree, max_edge_len, m_utilities=None):
    """
    Road map generation

    :param sample_points: x and y positions of sampled points in cell indices
    :param rr: Robot Radius
    :param obstacle_kd_tree: KDTree object of obstacles
    :param m_utilities: optional, needed when using costmap
    :return road_map

    Create a roadmap that is a list of sublists of tuples.
    Each sublist is a list of tuples corresponding to the indices of the nodes to which the sample point has an edge connection.
    e.g. for N_KNN=3:
    [[(i1, j1), (i2, j2), (i3, j3)], [(), (), ()], [(), (), ()], ...]
    Indicates that sample_point[0] is connected to 3 nodes, at indices (i1,j1), (i2,j2), (i3,j3).
    """

    sample_x = sample_points[0]
    sample_y = sample_points[1]

    # Note: The roadmap should have the same length as sample_points
    road_map = []
    n_sample = len(sample_x)

    #[Part 2] TODO Generate roadmap for all sample points, i.e. create the edges between nodes (sample points)
    # Note: use the is_collision function to check for possible collisions (do not make an edge if there is collision)
    # Hint: you may ceate a KDTree object to help with the generation of the roadmap, but other methods also work

    #create kd tree of samples
    samples = np.column_stack((sample_x, sample_y))
    sample_kdTree = KDTree(samples)

    # Note for this implementation, we obtain k nearest neighbours, and then only connect edges for one that don't cause collisions
    # therefore the actual number of connections made is <= k
    # iterate through all samples
    # note indices, mentioned in comments below, refer to indices of sample points in sample_x and sample_y (and samples), which are all parallel lists
    for sample_idx in range(n_sample):
        # list of indices of samples that current sample point will have edge to
        connections = findConnections(sample_kdTree, samples, sample_idx, rr, obstacle_kd_tree, max_edge_len)
        # add connection (edge) list to road map
        road_map.append(connections)

    return road_map


def plot_road_map(road_map, sample_points): 
    """
    Plot the generated roadmap by connecting the nodes with the edges as per generated roadmap
    """

    sample_x = sample_points[0]
    sample_y = sample_points[1]

    for i, _ in enumerate(road_map):
        for ii in range(len(road_map[i])):
            ind = road_map[i][ii]
            plt.plot([sample_x[i], sample_x[ind]],
                    [sample_y[i], sample_y[ind]], "-c")
    plt.title("PRM Graph")
    plt.xlabel("x (m)")
    plt.ylabel("y (m)")
    plt.xlim(0, 60)


def main(rng=None):
    print(__file__ + " start!!")

    # start and goal position
    sx = 10.0  # [m]
    sy = 10.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    robot_size = 5.0  # [m]

    ox = []
    oy = []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)


    obstacles = np.column_stack((ox, oy))

    prm_graph([sx, sy], [gx, gy], obstacles, robot_size, rng=rng)



if __name__ == '__main__':
    main()
