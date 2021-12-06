#!/usr/bin/env python
# coding: utf-8

# In[2]:


import networkx as nx
from matplotlib import pyplot as plt

#Initializing the position of the robot & the position of the pizzas that are appeared at random
robot_position = [(4,3)]
pizza_position = [(0,0), (9,0), (4,0)]


#Finding the Euclidean Distance between the robot and each of the pizzas
def euclideanDistance(coordinate1, coordinate2):
    return pow(pow(coordinate1[0] - coordinate2[0], 2) + pow(coordinate1[1] - coordinate2[1], 2), .5)


#Finding the pizza closest to the robot
def Closest_Pizza_Pos(robot_position, pizza_position):
    distances = []
    
    for j in range(len(pizza_position)):
        distances += [euclideanDistance(robot_position[0],pizza_position[j])]
    
    min_distance = min(distances)
    shortest_pizza_pos = distances.index(min_distance)
    
    #Returns the closest pizza
    return shortest_pizza_pos  

#Calling the function to find the nearest pizza
pizza_pos = Closest_Pizza_Pos(robot_position, pizza_position)


#Creating a 2D graph
Graph1 = nx.grid_2d_graph(10,4)

#Function to find the pizza without considering the obstacles
def Path_Finder(initial, final, Graph1):
    G=Graph1
    path = nx.bidirectional_shortest_path(G, source=initial, target=final)
    return path


#Robot's coordinates
initial = robot_position[0]
print('Position of the robot:', initial)
      
#Target pizza's coordinates
final = pizza_position[pizza_pos]
print('Position of the target pizza:', final)


#Defining a class to create nodes
class Node():
    #A node class for the A star Pathfinding

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position

        #g cost is the distance between the node
        #h cost is the heuristc value or the estimated value
        #f cost is defined to be gCost + hCost
        self.gCost = 0
        self.hCost = 0
        self.fCost = 0

    def __eq__(self, other):
        return self.position == other.position
    

#Defining A* algorithm
def A_star_algorithm(grid, start, end):
    
    #Creating a start and end node
    startNode = Node(None, start)
    startNode.gCost = startNode.hCost = startNode.fCost = 0
    endNode = Node(None, end)
    endNode.gCost = endNode.hCost = endNode.fCost = 0

    #Initializing open and closed lists
    openList = []
    closedList = []
    
    #Appending start node to open list
    openList.append(startNode)

    #Looping until we find the end node
    while len(openList) > 0:
        currentNode = openList[0]
        currentIndex = 0

        for index, item in enumerate(openList):
            if item.fCost < currentNode.fCost:
                currentNode = item
                currentIndex = index

        #Removing current index from open list and adding the current node to the closed list
        openList.pop(currentIndex)
        closedList.append(currentNode)

        #Checking if we have reached the final node
        if currentNode == endNode:
            path = []
            current = currentNode

            while current is not None:
                path.append(current.position)
                current = current.parent
            #Returning the reversed path
            return path[::-1] 

        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:

            node_position = (currentNode.position[0] + new_position[0], currentNode.position[1] + new_position[1])

            #Confirming the range
            if node_position[0] > (len(grid) - 1) or node_position[0] < 0 or node_position[1] > (len(grid[len(grid)-1]) -1) or node_position[1] < 0:
                continue

            #Confirming that the path has no obstacles 
            if grid[node_position[0]][node_position[1]] != 0:
                continue

            #Creating a new node
            new_node = Node(currentNode, node_position)
            children.append(new_node)
            
        for child in children:
            for closed_child in closedList:
                if child == closed_child:
                    continue

            #Creating the f, g, and h values
            child.gCost = currentNode.gCost + 1
            child.hCost = ((child.position[0] - endNode.position[0]) ** 2) + ((child.position[1] - endNode.position[1]) ** 2)
            child.fCost = child.gCost + child.hCost
            
            for openNode in openList:
                if child == openNode and child.gCost > openNode.gCost:
                    continue
                    
            openList.append(child)


#Initializing the map
grid =[[0, 0, 0, 0],
       [0, 0, 0, 0],
       [0, 1, 1, 0],
       [0, 0, 0, 0],
       [0, 1, 0, 0],
       [0, 1, 0, 0],
       [0, 0, 0, 0],
       [0, 0, 0, 0],
       [0, 1, 1, 0],
       [0, 0, 0, 0]]

#Defining start and end positions
StartNode = initial
EndNode = final

#Finding the path
path = A_star_algorithm(grid, StartNode, EndNode)
print('Shortest path: ', path)
           
            


# In[ ]:




