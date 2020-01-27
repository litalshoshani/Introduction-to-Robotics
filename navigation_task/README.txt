README

Background:
Robotic navigation is very important part of any robot mission. In the next exercise we used a map to navigate to places through the map. We did a smaller version of the navigation stack with the assumptions below. We used python and ros.

Assumptions:
We assumed the following about the environment and the robot's movement:
• Approximate cellular decomposition – the environment can be decomposed into a collection of uniform grid cells, each cell of size D. 
• The robot can move only in four compass directions - East, West, South and North.
• The robot has perfect localization. 
• The robot has a circular shape, whose diameter is equal to the cell size D. 
• The starting location will not be on an obstacle the goal can be on an obstacle. In this case the end goal should be in the last step of the path to the goal that is not an obstacle.

Input:
The algorithm is given the map of the environment as a .pgm file (as an argument in the launch file) and the robot diameter size (as a parameter in the launch file).

Algorithm:
The main steps of the algorithm are:
1). Load the occupancy grid map by calling the static_map service.
2). Switch to a grid, where each cell is equal to the size of the robot D. To avoid collisions of the robot with obstacles, if one of the sub-cells in a given grid's cell is occupied then mark the entire cell as occupied. 
3). Create a service that it’s request is two floats and it’s return is a boolean. The two floats represent the goal to reach in the world. It returns true when the robot reached it’s goal. 
4). Use an appropriate TF listener to find the robot's initial position in the map for each request. 
5). We use path planner - “find_path” (from previous exercise called “find_path”) to find a path from the current location to the goal location.
6). Smooth the path by choosing a set of waypoints on the path, such that the robot can move from one waypoint to the next one in a direct line (without turning). For example, if the path returned from A* is (1,1)→(1,2)→(1,3)→(2,3)→(2,4)→(2,5), and there is a direct line that connects (1,1) to (2,4) without getting into obstacles, then we can reduce the number of waypoints on the path to: (1,1)→(2,3)→(2,5). 
7). Send the appropriate cmd_vel commands to the robot to make it move along the waypoints (define a threshold which specifies how close the robot should get near a waypoint, before it can move to the next one). 
8). Wait for the next request.

Output:
Our application generates one output file: “new_grid.txt” - The new grid with D sized cells.

How to run the code:
run the commands:
“
catkin-make --pkg navigation_task
roslaunch find_path navigation_task.launch 
“
should be sufficient to start up Gazebo and the service. 
The next command in the terminal:
“
rosservice call /navigate '{x: point.x, y: point.y}'
“
(when point.x is the x value of the point and point.y is the y value of the point) makes the robot start the navigation and moving to the target.

The code is written by:
Lital Shoshani and Lotan Weiss.