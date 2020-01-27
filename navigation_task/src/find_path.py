#!/usr/bin/python
#
# find_path.py
#
#
import rospy
import sys
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid, Path


class find_path():

    '''
        initialization of the path finder
    '''
    def __init__(self, my_pmg, starting_location, goal_location, robot_size):
        self.my_pmg = my_pmg
        self.start = starting_location
        self.goal = goal_location
        self.bottomLeft = (my_pmg.info.origin.position.x,my_pmg.info.origin.position.y)
        self.robotSize = robot_size

        self.cell_size = self.my_pmg.info.resolution
        self.number_of_cell_for_robot = int(self.robotSize / self.cell_size) + 1

        self.grid = self.creat_occupancy_grid(self.my_pmg)
        self.new_grid = self.create_new_grid()


    '''
        writing grid to a file
    '''
    def print_map_to_file(self):
        with open("grid.txt", "w+") as grid_file:
            for row in reversed(self.grid):
                for cell in row:
                    grid_file.write("1") if cell else grid_file.write("0")
                grid_file.write("\n")

    def print_new_grid_to_file(self):
        with open("new_grid.txt", "w+") as new_grid_file:
            for row in reversed(self.new_grid):
                for cell in row:
                    new_grid_file.write("1") if cell else new_grid_file.write("0")
                new_grid_file.write("\n")

    '''
        creating occupancy grid
    '''
    def creat_occupancy_grid(self, my_map):
        # creating the occupancy grid
        grid = [[None] * my_map.info.width for i in xrange(my_map.info.height)]
        for i in xrange(my_map.info.height):
            for j in xrange(my_map.info.width):
                if my_map.data[i * my_map.info.width + j] == 0:
                    grid[i][j] = False
                else:
                    grid[i][j] = True
        return grid

    '''
        creating new grid such that each cell is the size of the robot
    '''
    def create_new_grid(self):

        newHeight = int(self.my_pmg.info.height/self.number_of_cell_for_robot)
        newWidth = int(self.my_pmg.info.width/self.number_of_cell_for_robot)

        rowsrun = self.my_pmg.info.height - self.number_of_cell_for_robot
        colsrun = self.my_pmg.info.width - self.number_of_cell_for_robot

        new_grid_i = 0
        new_grid_j = 0

        # initialization
        new_grid = [[None] * newWidth for e in range(newHeight)]

        # placing true values according to original grid
        for i in range(0,rowsrun,self.number_of_cell_for_robot):
            for j in range(0,colsrun,self.number_of_cell_for_robot):
                if (self.find_obstacle_by_nocfr(self.number_of_cell_for_robot,i,j)):
                    new_grid[new_grid_i][new_grid_j] = 1
                else:
                    new_grid[new_grid_i][new_grid_j] = 0

                new_grid_j += 1
            new_grid_j = 0
            new_grid_i += 1
        return new_grid


    '''
        checks if cell is obstacle in the grid by robot size
        (nocfr = number of cell for robot)
    '''
    def find_obstacle_by_nocfr(self,number_of_cell_for_robot,startI,startJ):
        for i in range(number_of_cell_for_robot):
            for j in range(number_of_cell_for_robot):
                if (self.grid[startI+i][startJ+j] == 1):
                    return True
        return False


    '''
        create a file with the desired path
    '''
    def print_path(self):
        solution = self.find_shortest_path()
        with open("path.txt", "w+") as path_file:
            for e in solution:
                path_file.write("(" + str(e.x) + "," + str(e.y) + ")\n")


    '''
        finding the shortest path from start to goal
    '''
    def find_shortest_path(self):
        closedset = set()
        solution = []
        myQ = []
        startPoint = self.getNewPointFromOrigin(self.start[0],self.start[1])
        goalPoint = self.getNewPointFromOrigin(self.goal[0],self.goal[1])

        myQ.insert(0,Node(startPoint[0],startPoint[1]))
        while (myQ != []):
            curr = myQ.pop()
            if (curr.x == goalPoint[0] and curr.y == goalPoint[1]):
                if (self.new_grid[curr.x][curr.y] == 1):
                    solution = self.getSolution(curr.cameFrom)
                else:
                    solution = self.getSolution(curr)
                break
            myNeighbors = self.getNeighbors(curr.x,curr.y,goalPoint)
            for neighbor in myNeighbors:
                if ((neighbor.x,neighbor.y) not in closedset):
                    closedset.add((neighbor.x,neighbor.y))
                    neighbor.cameFrom = curr
                    myQ.insert(0,neighbor)
        solution.reverse()
        return solution

    '''
        gets node's neighbors
    '''
    def getNeighbors(self,x,y,goalPoint):
        neighbors = []

        # seeking for non-obstacle neighbor
        if (x>0):
            if (self.new_grid[x-1][y] == 0):
                neighbors.append(Node(x-1,y))
        if (y+1<len(self.new_grid[0])):
            if (self.new_grid[x][y+1] == 0):
                neighbors.append(Node(x,y+1))
        if (x+1<len(self.new_grid)):
            if (self.new_grid[x+1][y] == 0):
                neighbors.append(Node(x+1,y))
        if (y>0):
            if (self.new_grid[x][y-1] == 0):
                neighbors.append(Node(x,y-1))

        # seeking for goal neighbor
        if (x>0):
            if (self.new_grid[x-1][y] == 1 and x-1 == goalPoint[0] and y == goalPoint[1]):
                neighbors.append(Node(x-1,y))
        if (y+1<len(self.new_grid[0])):
            if (self.new_grid[x][y+1] == 1 and x == goalPoint[0] and y+1 == goalPoint[1]):
                neighbors.append(Node(x,y+1))
        if (x+1<len(self.new_grid)):
            if (self.new_grid[x+1][y] == 1 and x+1 == goalPoint[0] and y == goalPoint[1]):
                neighbors.append(Node(x+1,y))
        if (y>0):
            if (self.new_grid[x][y-1] == 1 and x == goalPoint[0] and y-1 == goalPoint[1]):
                neighbors.append(Node(x,y-1))

        return neighbors


    '''
        finding path by backtracking the goal node
    '''
    def getSolution(self,goalNode):
        solution = []
        curr = goalNode
        while (curr.cameFrom != None):
            solution.append(curr)
            curr = curr.cameFrom
        solution.append(curr)
        return solution

    '''
        finding points on new grid by thier origin value
    '''
    def getNewPointFromOrigin(self,x,y):
        origX = self.bottomLeft[0]
        origY = self.bottomLeft[1]
        i = int(((x-origX)/self.cell_size)/self.number_of_cell_for_robot)
        j = int(((y-origY)/self.cell_size)/self.number_of_cell_for_robot)

        return (i,j)

    '''
        compressing path to critical points
    '''
    def smooth_path(self,solution):
        if (len(solution) <= 1):
            return {}, solution

        smoothed_solution = [solution[0]]
        runX = False
        runY = False

        dct = {}
        tmp = solution[0]

        for i in range(1, len(solution)):
            if (not runX and not runY):
                if (solution[i].x != solution[i - 1].x):
                    runX = True
                else:
                    runY = True
                # tmp = solution[i-1]
                # smoothed_solution.append(tmp)
            elif (runY and solution[i].x != solution[i - 1].x):
                smoothed_solution.append(solution[i])
                dct[tmp] = solution[i - 1]
                runY = False
                tmp = solution[i]
            elif (runX and solution[i].y != solution[i - 1].y):
                smoothed_solution.append(solution[i])
                dct[tmp] = solution[i - 1]
                runX = False
                tmp = solution[i]
        if (runX or runY):
            smoothed_solution.append(solution[-1])
            dct[tmp] = solution[-1]
        dct[solution[-1]] = None

        return smoothed_solution, dct



# helpful class for bfs
class Node():
    def __init__(self,x,y):
        self.x = x
        self.y = y
        self.cameFrom = None

# helps to parser string input
def input_parser(strRepr):
    args = strRepr.split(",")
    return (float(args[0]),float(args[1]))



