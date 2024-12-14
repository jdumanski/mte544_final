import matplotlib.pyplot as plt
from utilities import FileReader




def plot_errors():
    
    headers, robotPosesPRM=FileReader("trial1/robotPose_PRM_t1.csv").read_file()
    headers, idealPathPRM=FileReader("trial1/idealPath_PRM_t1.csv").read_file()
    headers, robotPosesGrid=FileReader("trial1/robotPose_grid_t1.csv").read_file()
    headers, idealPathGrid=FileReader("trial1/idealPath_grid_t1.csv").read_file()
    headers, obstacles=FileReader("obstacles.csv").read_file()
    #time_list=[]
    
    #first_stamp=values[0][-1]
    
    #for val in values:
    #    time_list.append(val[-1] - first_stamp)



    #for i in range(0, len(headers) - 1):
    #    plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")
    x_r_PRM, y_r_PRM = zip(*robotPosesPRM)
    x_p_PRM, y_p_PRM = zip(*idealPathPRM)

    x_r_grid, y_r_grid = zip(*robotPosesGrid)
    x_p_grid, y_p_grid = zip(*idealPathGrid)

    x_o, y_o = zip(*obstacles)

    plt.plot(x_r_PRM, y_r_PRM, label="Robot pose following PRM + A*")
    plt.plot(x_p_PRM, y_p_PRM, label="Generated PRM + A* path")
    plt.plot(x_r_grid, y_r_grid, label="Robot pose following grid + A*")
    plt.plot(x_p_grid, y_p_grid, label="Generated grid + A* path")
    plt.scatter(x_o, y_o)
    plt.plot(x_p_PRM[0], y_p_PRM[0], "^r")
    plt.plot(x_p_PRM[-1], y_p_PRM[-1], "^g")

    plt.xlabel("x(m)")
    plt.ylabel("y(m)")
    plt.title("Grid A* vs PRM A*")
    plt.legend()
    plt.grid()

    plt.show()
    
    

if __name__=="__main__":
    plot_errors()