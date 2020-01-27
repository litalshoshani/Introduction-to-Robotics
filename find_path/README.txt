README

Background:
Robotic navigation is very important part of any robot mission. In the next exercise we used a map to navigate to places through the map. We used python and ros.

Assumptions:
We assumed the following about the environment and the robot's movement:
• Approximate cellular decomposition – the environment can be decomposed into a collection of uniform grid cells, each cell of size D. 
• The robot can move only in four compass directions - East, West, South and North.
• The robot has perfect localization. 
• The robot has a circular shape, whose diameter is equal to the cell size D. 
• The starting location will not be on an obstacle the goal can be on an obstacle. In this case the end goal should be in the last step of the path to the goal that is not an obstacle.

Input:
The algorithm is given the map of the environment as a .pgm file (as an argument in
the launch file) and the robot diameter size (as a parameter in the launch file) and the
starting and end location of the robot (also given as a parameter in the launch file).

Algorithm:
The main steps of the algorithm are:
1. Load the occupancy grid map by calling the static_map service.
2. Switch to a grid, where each cell is equal to the size of the robot D. To avoid collisions of the robot with obstacles, if one of the sub-cells in a given grid's cell is occupied then mark the entire cell as occupied.
3.Find all the free cells that are reachable from the robot's initial position and ignore all the other cells.
4. Print a path of grid cells (in the D size grid) from the starting location to the end location. In order to find this path we used BFS search algorithm.

Output:
Our application generates two output files: 
“new_grid.txt” - The new coarse grid.
path.txt – the path from the starting position to the goal.
(see directory "example_file")

How to run the code:
run the commands:
“
catkin-make --pkg find_path
roslaunch find_path navigation.launch 
“
should be sufficient to start up Gazebo and make the robot start the navigation
process.

The code is written by:
Lital Shoshani and Lotan Weiss.
