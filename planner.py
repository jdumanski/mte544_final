from mapUtilities import *
from a_star import *
from probabilistic_road_map import *
import time

POINT_PLANNER=0; TRAJECTORY_PLANNER=1; ASTAR_PLANNER=2; PRM_PLANNER=3

ROBOT_RADIUS = 0.32 #in meters

class planner:
    def __init__(self, type_, mapName="room"):
        self.type=type_
        self.mapName=mapName
        self.hasPRM = False
    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)

        self.costMap=None
        self.initTrajectoryPlanner()
        
        return self.trajectory_planner(startPose, endPose, self.type)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):
        
        #### If using the map, you can leverage on the code below originally implemented for A*
        self.m_utilities=mapManipulator(laser_sig=0.4)    
        self.costMap=self.m_utilities.make_likelihood_field()

        # List of obstacles to plot (in x and y coordiantes)
        self.obstaclesList = np.array(self.m_utilities.getAllObstacles())     

        # List of obstacles for PRM, in cell indices
        self.obstaclesListCell = np.array(self.m_utilities.getAllObstaclesCell())  

        
    def trajectory_planner(self, startPoseCart, endPoseCart, type):

        startPose=self.m_utilities.position_2_cell(startPoseCart)
        endPose=self.m_utilities.position_2_cell(endPoseCart)

        # [Part 3] TODO Use the PRM and search_PRM to generate the path
        # Hint: see the example of the ASTAR case below, there is no scaling factor for PRM
        if type == PRM_PLANNER:
            # if prm has already been generated, no need to generate a new one
            # just add new start and end points to existing prm
            if self.hasPRM:
                print("Replanning without creating new PRM")
                # add start and end points to end of prm points list
                self.points.append(startPose)
                self.points.append(endPose)
                # reconstruct objects needed to find connections of start and end nodes in prm graph
                obstacle_kdtree = KDTree(self.obstaclesListCell)
                sample_kdtree = KDTree(self.points)
                startIdx = len(self.points)-2
                endIdx = len(self.points)-1
                # find connections to start node (using same KNN, max_edge_len, and collision criteria used to construct prm originally)
                startConnections = findConnections(sample_kdtree, self.points, startIdx, ROBOT_RADIUS, obstacle_kdtree)
                # add connections to prm to start node now connected in prm
                self.prm.append(startConnections)
                # find connections to end node
                endConnections = findConnections(sample_kdtree, self.points, endIdx, ROBOT_RADIUS, obstacle_kdtree)
                # add connections to prm to end node now connected in prm
                # note we add edges (connections) from each neighbour to the end node, as opposed to egdes from the end node
                # to each neighbour, since we need edges going towards the end node to form a path (since edges are directed in
                # this implementation since the prm is an adjacency list)
                for connection in endConnections:
                    self.prm[connection].append(endIdx)
            else:
                # if prm does not exist, generate one
                self.hasPRM = True
                self.points, self.prm = prm_graph(startPose, endPose, self.obstaclesListCell, ROBOT_RADIUS, m_utilities=self.m_utilities)
            # time A* search on prm
            start_time = time.time()
            # execute search on prm
            path_ = search_PRM(self.points, self.prm, startPose, endPose)
            end_time = time.time()
            print(f"the time took for a_star calculation on PRM was {end_time - start_time}")

        elif type == ASTAR_PLANNER: # This is the same planner you should have implemented for Lab4
            scale_factor = 4 # Depending on resolution, this can be smaller or larger
            startPose = [int(i/scale_factor) for i in startPose]
            endPose   = [int(j/scale_factor) for j in endPose]
            start_time = time.time()

            path = search(self.costMap, startPose, endPose, scale_factor)

            end_time = time.time()

            print(f"the time took for a_star calculation on grid was {end_time - start_time}")

            path_ = [[x*scale_factor, y*scale_factor] for x,y in path ]

        Path = np.array(list(map(self.m_utilities.cell_2_position, path_ )))

        # Plot the generated path
        plt.plot(self.obstaclesList[:,0], self.obstaclesList[:,1], '.')
        plt.plot(Path[:,0], Path[:,1], '-*')
        plt.plot(startPoseCart[0],startPoseCart[1],'*')
        plt.plot(endPoseCart[0],
                 endPoseCart[1], '*')

        plt.show()
        
        return Path.tolist()
    

if __name__=="__main__":

    m_utilities=mapManipulator()
    
    map_likelihood=m_utilities.make_likelihood_field()
    
    # You can test your code here...
