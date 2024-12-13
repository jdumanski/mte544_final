# Final exam (30% of the final total grade)

## Introduction

In the final exam, you will work on an individual project that is based on the same code structure as the ones used for the labs. 

**Make sure you familiarize yourself with the entire structure before attempting this exam**.

If you haven't, you can also check the PowerPoint presentation from the lecture on Dec. 3rd where we explained the final exam and the expectations.

**Rules for the final exam:**
- It must be completed individually, do not work in groups, and do not violate Policy 71 (you can reuse material from your previous labs);
- Do not ask questions via email, use Piazza;
- You may ask questions on Piazza both publicly and privately, private questions are allowed only if identity cannot be hidden (e.g. screenshot with your name), or if there are pieces of solutions, private questions that do not contain these information will not be answered until they are made public;
- You may request individual meetings with the teaching team during the exam period (Dec. 6 - Dec. 13), meetings should not exceed 15 minutes;
- In case of emergency, you may request resources (laptops, remote desk computers) by emailing your TA Amr Hamdi (amhamdi@uwaterloo.ca), no resources will be allowed to be requested after 5PM Dec. 13.

Please note that both questions and individual meetings must be given sufficient time to be answered and planned (meetings should be requested with at least 12 hours notice, preferably). After 5PM Dec. 13, questions are not guaranteed answers, and meetings will not be planned.

**Ask questions on Piazza**

https://piazza.com/uwaterloo.ca/fall2024/mte544

**Individual meetings**

You may contact the following people (please add everyone so the first available person can reply) for an in-person or online meeting:
- Pi (tchoopoj@uwaterloo.ca)
- Minghao (minghao.ning@uwaterloo.ca)
- Prof. Hu (yue.hu@uwaterloo.ca)

**Emergency resources**
- Amr Hamdi (amhamdi@uwaterloo.ca)

**Environment for the final exam**

For the final exam, no real robot is needed, everything will be conducted in simulation. Refer to `tbt3Simulation.md` in the `main` branch to see how to launch the Gazebo simulation. If you installed your environment using the script provided in `setup`, then it should work without any issues. Likely, you should have checked it's fully functional during the term for your labs.

For the final exam, you will run Gazebo in the house environment.

```
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

**Grading**: refer to ```rubrics.md``` for detailed marking scheme.

## Part 1 - Explanation of the stack and flow-chart (15 marks)
Throughout labs 2 to 4, you've been working on the same stack that comprises control, state estimation, and path planning.
In this final exam, you will implement a new planning algorithm based on the Probabilistic Roadmap (PRM), see the detailed steps in the following parts.

Read thoroughly the entire manual first, then:
- Create a flow chart using a digital graphics tool (e.g. https://app.diagrams.net/) that illustrates how the entire stack works, including the parts to be added for this final exam. Specify which information/data and how they are exchanged between each block. You may base this on the chart shown during the tutorials, but it must include the parts required for this final exam.

Then, in a maximum of 500 words, explain how the stack works with reference to the flow-chart and the several classes involved:
- ```decisions.py```
- ```planner.py```
- ```localization.py```
- ```controller.py```

In the explanation:
- Refer back to what you've done in the labs and how you progressed each lab towards the complete stack.
- Indicate how the new portion of the final exam is incorporated.
- Make connections to the theoretical content learned during the lectures.

## Part 2 - Implement the PRM algorithm (35 marks)
In Lab 4, you used the A* algorithms on an occupancy grid map, which is a deterministic way to obtain the graph. 
In this final exam, instead of using the occupancy grid directly as a graph, you are asked to implement the Probabilistic RoadMap (PRM) algorithm to generate a graph based on random sampling.

Complete the provided code of the PRM algorithm `probabilistic_road_map.py` and visualize the obtained graph. You can run this independently outside of the stack, see detailed instructions and the algorithm explained on the top of the Python file.

You will need to complete portions of the following four functions:
- `prm_graph` which sets the parameters and calls the functions to generate the points and the roadmap.
- `generate_sample_points`, which samples the points (nodes) in the free space.
- `generate_road_map`, which generates the connected graph (roadmap) from the samples.
- `is_collision`, which checks if there is a collison when connecting nodes.

Properly comment your code.

For testing purposes, a list of obstacles and starting and goal points are given in the main function. You can run the Python file indepdendently without the stack, remember to set the flag `use_map` to False when doing so. Run the algorithm with:
```
python3 probabilistic_road_map.py
```
You should obtain a visualization of the roadmap. Generate two roadmaps to add to your report.

## Part 3 - Combine the PRM algorithm with A* path planning (15 marks)

To do this, you are provided with a map of the TurtleBot3 world that was acquired beforehand, see `room.pgm` and `room.yaml`.
Integrate the PRM into the stack such that you can search the obtained graph using A* to obtain a path and execute the resulting path on the robot in simulation. 

The A* you had used for Lab4 was ad-hoc implemented for a grid search. A new function `seach_PRM` is mostly implemented for you in `astar.py`.

To complete this part:
- Complete the code in `search_PRM` so that the A* search is performed on the generated roadmap.
- Complete the code in `planner.py` so that the PRM graph generator is integrated with the A* search, the controller, and the state estimator.
- Add any other code needed to have a working integration.

Properly comment your code.

## Part 4 - Test the stack with PRM and A* in simulation (35 marks)
Test your stack in the Gazebo simulator, and make sure to test different parameters of PRM and A* to evaluate their effects.
Please note that the map is previously acquired, therefore you need to reset the pose of the robot at the beginning of each trial. (See Bonus section for real-time adaptation)

To test your PRM + A*:
- Use the Extended Kalman Filter as the localizer (change in ```decisions.py```), use your tuned covariances, or tune them in simulation. Tune the PID gains as well, if needed. We will not deduct marks for poor localization or path tracking performance.
- Choose a goal pose: you can use the same RViz interface as the one for Lab4 (see below) to choose a goal pose, or you can hard-code a goal pose.
- Execute the path in Gazebo.

**To choose a goal pose in RViz**
(Remember to run the map publisher with `python3 mapPublisher.py`)
- In a terminal run the rviz2 with the given configuration: ```rviz2 -d pathPlanner.rviz```
- In another terminal run the decisions.py: ```python3 decisions.py```
- On rviz2 use the 2D goal pose on the toolbar on top to choose the goal pose
- Watch the robot go to the specified point (if plots are on, you will need to close the matplotlib windows before the robot starts the execution of the path)

Show your results with at least **two different goal points**. To report your results:
- Tune your PRM and A* parameters, in the written report you will be asked to discuss this process.
- Plot containing the graph generated with PRM and the obtained path, clearly marking the starting and goal positions. Overlay the generated path with the executed path.

Compare the PRM+A* with A* on a grid (Lab4):
- Compare the PRM+A* planner you implemented with the provided A* search on a grid map.
- In `decisions.py`, change the planner to the ASTAR_PLANNER and execute the planning for the same goal points as above.
- Plot the generated graph and the obtained path, clearly marking the starting and goal positions. Overlay the generated path with the executed path.
- Compare in terms of total cost (path length) and computational time, report these numbers in the report and discuss your observations in terms of pros and cons.

Record a video of at least one successful trial, the video should contain:
- A clear view of the terminal(s) with your username (see the video in the FinalExam power point presentation on Learn);
- The choice of the goal point from RViz;
- The plot of the generated PRM;
- The plot of the path obtained from A*;
- The beginning of the path execution in Gazebo (green path in RViz and robot moving in Gazebo), no need to record the entire execution, but record sufficient time to see that the robot is moving and following the path.
The path shown in the video must be one of the plots in the report. Clearly state in the report which one corresponds to the video.

**Video recording guidelines**

Do not use your phone or an external camera pointing at the computer screen for the video, use a screen capture/recorder (e.g. on Windows 11 you can use the snipping tool, in Ubuntu you can use the screenshot tool).

## Bonus - online replanning with new obstacles (10 marks)
**Note: there will be no support provided for this part**

For this part, modify the code so that new obstacles that appear along the current path can be accounted for during the navigation.
This means:
- PRM needs to be regenerated when new obstacles are encountered;
- A new path (globally or locally) needs to be recomputed accordingly;
- Simultaneous Localization and Mapping (SLAM) is needed, you can use the Nav2 package;
- You need to add new obstacles (at least one) to the house environment, you can do this in Gazebo after launching it, using one of the existing geometric shapes (cube or cylinder). Make sure to add the obstacle on the current path so that replanning is necessary.

You may use the provided map of the house to start with and then run SLAM for refining.
To demonstrate success, in addition to the requirements from Parts 1 to 4:
- Provide the map with added obstacles, clearly indicating the location of these obstacles.
- Provide the path the robot originally planned and the one it eventually executed due to replanning.
- Record a video (with the same requirement as Part 4), where the new obstacle is added while the robot is executing the current path, to showcase the replanning during navigation.

## Report Format
Please prepare a written report adhering to the following report format to help with organization and to ensure we do not miss content when marking your work. **Penalties will be applied if the following report format is not followed (see rubrics.md).**

**Title Page:** 

- Name (Family Name, First Name);
- Student ID;
- Whether you have attempted the bonus points.

**Report Content:** 

Make sure to check `rubrics.md` for the related marks.

* Part 1 - Stack: the flowchart
  * Flowchart of the stack.
  * The description and discussion of the stack, max 500 words.
* Part 2 - PRM implementation and A* integration: 
  * Results
    * Report figures of your 2 roadmaps.
  * Analysis
    * Describe how the provided PRM works and which modifications you implemented.
    * Describe how you integrated PRM with A* planning and the rest of the stack for the robot from the planner to the actual motions of the robot (you do not need to describe the entire stack again as you should have done this in Part 1).
* Part 4 - Discussion, Testing, and Integration of PRM+A*: 
  * Results
    * Discuss how you tuned your PRM and A* parameters to reach the final parameters you used for the plots (you may use figures for support if needed).
    * Report plots for PRM+A* and A* for two goal poses on a grid as specified in Part 4.
  * Analysis
    * Discuss the performance of your path planner against the original A* on the occupancy grid map (Lab 4).
    * Discuss the overall performance of your entire stack, i.e. including your PID controller, EKF, and PRM + A* planner.
* Appendix - Code
  * Copy all the code you have developed for all the TODO points inside the Appendix (no page limits), no screenshot, the text within the PDF must be detected as text or penalty will apply.
    * Create a section for each of the parts 1 to 4 and paste the code of the TODO points of each part in the respective sections.

The text and plots for the bonus are expected to be integrated within the specified sections, even if it is only partially attempted (not functional or incomplete).

**Page limits (excluding the front page and appendix):**
- 5 pages;
- 6 pages if with bonus marks (new obstacles and replanning).
- Appendix: no limit, but the report should be self-contained (appendix should not contain fundamental information that can impact grading).

## Submission

Submit the report and the code on Dropbox (LEARN) in the corresponding folder.
- **Report**: One single pdf with the file name `{Family Name}-{First Name}-MTE544-Final-Report` that can be checked for Turnitin (text must be detectable as such, i.e. not printed pdfs);
- **Code**: Make sure to have commented your code! Submit one single zip file named `{Family Name}-{First Name}-MTE544-Final-Code` with everything (including the csv files obtained from the data log and the map files).
- **Video**: video(s) file or link to the video(s) in the report.


Good luck!





