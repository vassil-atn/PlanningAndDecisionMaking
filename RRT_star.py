import numpy as np
from robotModel import Robot
from shapely.geometry import Polygon,Point
from collision_detection import checkPolysIntersecting
import matplotlib.pyplot as plt
from matplotlib.patches import Ar
from RRT_algorithm import collisionFree,findDistance,Node

# This function finds the nearest nodes around the new node in a given radius
def findNearestNodes(q,NodeList):
    nearest_nodes = []
    radius = 3 # Define the radius in which to check for more optimal paths
    for idx,node in enumerate(NodeList):
        if findDistance(node,q) < radius:
            nearest_nodes.append(idx)
    return nearest_nodes
# This function finds the lowest cost node in the vicinity of the new node
# to be selected as a candidate for its parent node
def chooseParent(NodeList,nearest_nodes,q):
    min_cost = float('inf')
    for i in enumerate(nearest_nodes):
        cost = NodeList[nearest_nodes[i]].cost + findDistance(NodeList[nearest_nodes[i]],q)
        if  cost < min_cost:
            min_cost = cost
            parent_node = NodeList[nearest_nodes[i]]
            parent_index = nearest_nodes[i]
    return parent_node,parent_index
#
#
# This function updates the costs of leaves once their parent node's cost has been updated
def changeLeavesCost(rewired_node,NodeList): 
    for node in NodeList:
        if node.parent == rewired_node:
            node.cost = rewired_node.cost + findDistance(rewired_node,node)
        

def RRT_star(start,goal,N):
    NodeList = []
    plt.figure(1)
    r = Robot()
    # First add the starting node:
    Node_inst = Node(start)
    NodeList.append(Node_inst)
    # Define sets for each configuration variable:
    q1_set = np.array([0,10])
    q2_set = np.array([0,10])
    q3_set = np.array([0,2*np.pi])
    q4_set = np.array([0,2*np.pi])
    q5_set = np.array([0,2*np.pi])
    
    for i in range(0,N):

        print(f'Running sample number {i}')

        # Pick a random point in C-space
        q = np.array([np.random.uniform(q1_set[0],q1_set[1]),
                      np.random.uniform(q2_set[0],q2_set[1]),
                      np.random.uniform(q3_set[0],q3_set[1]),
                      np.random.uniform(q4_set[0],q4_set[1]),
                      np.random.uniform(q5_set[0],q5_set[1])])
        
        if collisionFree(r,q) == True:
            nearest_nodes = findNearestNodes(q,NodeList)
            for i in enumerate(nearest_nodes):
                # Choose the best candidate for the parent node:
                parent_node,parent_index = chooseParent(NodeList,nearest_nodes,q)
                # NEED TO CREATE THIS FUNCTION WITH THE NEW STEERING FN:
                if collisionFreePath(parent_node,q): # Here check if the path is collision free
                    Node_inst = Node(q)
                    Node_inst.parent = parent_node
                    Node_inst.cost = Node_inst.parent.cost + findDistance(Node_inst.parent,q)
                    NodeList.append(Node_inst)
                    break
                else:
                    nearest_nodes.remove(parent_index) # remove the best candidate from the nearest_nodes
                    # and check the next one
        
            
        # Check if any of the nearby nodes need to be updated due to the new node:
        nearest_nodes = findNearestNodes(q,NodeList) 
        for i in enumerate(nearest_nodes):
            dist = findDistance(NodeList[-1],NodeList[i]) # distance between new node and nearest_nodes
            if  (dist + NodeList[-1].cost) < NodeList[i].cost:
                # If the cost through the new node to some nearest node is lower than that node's
                # original cost - update it
                NodeList[i].parent = NodeList[-1]
                NodeList[i].cost = dist + NodeList[-1].cost
                # Recursively change the cost of each leaf of the reconnected node
                changeLeavesCost(NodeList[i],NodeList)
                
        # CHECK THE GOAL CONDITION:
        
        # if the goal is close to the last added node        
        #if (abs(findDistance(NodeList[-1].q,goal)) < 5):
        # Define path from last added node to goal:
        storeModel,r = steeringFunction(NodeList[-1].q,goal)
        # Check if path is free:
        for n in range(len(storeModel)):
            if collisionFree(r,storeModel[n])==0:
                freePath = False
                break
            freePath = True
            
        if freePath == True:
            Node_inst = Node(goal)
            Node_inst.parent = NodeList[-1]
            NodeList.append(Node_inst)
            break

    print("Path successfuly found!")
    return NodeList