import numpy as np
from robotModel import Robot
from shapely.geometry import Polygon,Point
from collision_detection import checkPolysIntersecting
import matplotlib.pyplot as plt

class Node:
    def __init__(self,q):
        self.q = q
        self.parent = None


def HaarMeasure(angle1,angle2):
    # First bound the angles to be between 0 and 2*pi
    angle1 = angle1 % (2*np.pi) 
    angle2 = angle2 % (2*np.pi) 
    # Then find the absolute difference
    absolute = abs(angle1-angle2)
    # Use min function because you can move around the circle clockwise and counterclockwise:
    return min(absolute,2*np.pi-absolute)


def findDistance(q1,q2):
    # Use Haar Measure for the angles:
    distance = np.linalg.norm(q1[0:2] - q2[0:2]) + HaarMeasure(q1[2],q2[2]) + HaarMeasure(q1[3],q2[3])
    
    return distance

def steeringFunction(q1,q2):
    # convert q2 to workspace coordinates (x,y,theta), feed into IK model to find the path
    # MODIFY 
    storeModel = []
    #mp,p_centre1,p_centre2,joint_1,joint_2 = r.ForwardKinematicsConfig(q1)
    r = Robot(mp=np.array([q1[0],q1[1]]),phi=q1[2],q=np.array([q1[3],q1[4]]))
    
    mp,p_centre1,p_centre2,joint_1,joint_2 = r.ForwardKinematicsConfig(q2)
    
    
    
    X_des = np.array([joint_2[0],joint_2[1],np.sum(q2[2:5])])
    T = 2
    dt = 0.001
    prev_error = 0
    error_i = 0

    for i in range(0,int(T/dt)):
    
        # PID CONTROLLER:
        Kp = 20
        Ki = 0.1
        Kd = 0.001
        
        
        # Inverse Dynamics:
    
        J = r.Jacobian(r.u)    
        J_inv = np.dot(J.T,np.linalg.inv(np.dot(J,J.T)))
        
        X = np.array([r.p[0],r.p[1],r.theta])
        
        error = X_des - X
        error_d = (prev_error - error)
        error_i = error_i + error
        
        # Basically x_dot = K*(x_des - x) (plus the integral and derivative terms)
        X_dot_controller = Kp*error + Ki*error_i*dt + Kd*error_d/dt
        
        Q_dot_des = np.dot(J_inv,X_dot_controller)
        
        # Get the desired inputs for the joints and wheels:
        u = Q_dot_des[0:2]
        dq = Q_dot_des[2:4]
        
        phi_dot = (u[1] - u[0])/(2*r.h)
        mp_dot = np.array([np.cos(r.phi)*np.sum(u[:])/2, np.sin(r.phi)*np.sum(u[:])/2])
        
        # Simulate forward motion with these desired commands:
        phi_dot = (u[1] - u[0])/(2*r.h)
        X_dot = r.ForwardKinematics(u,dq,phi_dot)
        
        mp_dot = np.array([np.cos(r.phi)*np.sum(u[:])/2, np.sin(r.phi)*np.sum(u[:])/2])
        # Integrate numerically:
        mp = r.mp.copy()
        phi = r.phi
        p = r.p.copy()
        theta = r.theta
        q = r.q.copy()
            
        mp += mp_dot*dt
        phi += phi_dot*dt
 
        q += r.dq[:]*dt
        p[0] = mp[0] + np.cos(phi)*(r.l[0]*np.cos(q[0]) + r.l[1]*np.cos(q[0]+q[1])) - np.sin(phi)*(r.l[0]*np.sin(q[0]) + r.l[1]*np.sin(q[0]+q[1]))
        p[1] = mp[1] + np.sin(phi)*(r.l[0]*np.cos(q[0]) + r.l[1]*np.cos(q[0]+q[1])) + np.cos(phi)*(r.l[0]*np.sin(q[0]) + r.l[1]*np.sin(q[0]+q[1]))
        theta = phi + q[0] + q[1]
        
        # Position of the first joint w.r.t the mobile base:
        p_em = np.array([r.l[0]*np.cos(q[0]),
                         r.l[0]*np.sin(q[0])])
        p_joint_1 = np.zeros(2)
        p_joint_1[0] = mp[0] + np.cos(phi)*p_em[0] - np.sin(phi)*p_em[1]
        p_joint_1[1] = mp[1] + np.sin(phi)*p_em[0] + np.cos(phi)*p_em[1]
        
        r.Update(mp,phi,p,theta,q,dq,u,p_joint_1) 
        
        # Save error for the derivative controller
        prev_error = error
        
        storeModel.append(np.array([mp[0],mp[1],phi,q[0],q[1]]))
        
    return storeModel,r

def collisionFree(r,q,obstacles=None):

    # Build the bounding boxes for the base and the links:
    poly_base,poly_link1,poly_link2 = collisionBox(r,q)
    
    # Check if in collision:
    obstacles = [Polygon([np.array([1,1]),np.array([1,2]),np.array([2,1]),np.array([2,2])])] 
    collision_free = True
    if obstacles != None:
        for obst in obstacles:
            if poly_base.intersects(obst) or poly_link1.intersects(obst) or poly_link2.intersects(obst):
                collision_free = False
                break
# =============================================================================
#             if checkPolysIntersecting(poly_base,obst): 
#                 collision_free = False
#                 break
#             if checkPolysIntersecting(poly_link1,obst): 
#                 collision_free = False
#                 break
#             if checkPolysIntersecting(poly_link2,obst): 
#                 collision_free = False
#                 break
# =============================================================================
        
    return collision_free

#
#
# Helper function to define the vertices of the polygons around the robot arm
def buildPolygons(r,link_centre,angle): 
    poly = []
    link_length = r.l[0]
    link_width = 0.2
    r = Robot()
    R = r.rotationMatrix(angle)
    poly.append(link_centre + R.dot(np.array([-link_length/2,link_width/2])))
    poly.append(link_centre + R.dot(np.array([link_length/2,link_width/2])))
    poly.append(link_centre + R.dot(np.array([link_length/2,-link_width/2])))
    poly.append(link_centre + R.dot(np.array([-link_length/2,-link_width/2])))
    return poly
#
#
#
# This function finds the collision polygons for the robot (base+arms)
def collisionBox(r,q): 
    mp,p_centre1,p_centre2,_,_ = r.ForwardKinematicsConfig(q)
    phi = q[2]
    # for the base it's a circle:
    base_box = Point(mp).buffer(0.6) # Create a circle centered at the centre of the mobile base with radius 0.5
    polygon1 = Polygon(buildPolygons(r,p_centre1,r.q[0]+phi))
    polygon2 = Polygon(buildPolygons(r,p_centre2,q[3]+q[4]+phi))
    
    return base_box,polygon1,polygon2
#
#   
#
def RRT(start,goal,N=100):
    NodeList = []
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
        best_distance = float('inf')
        # Pick a random point in C-space
        q = np.array([np.random.uniform(q1_set[0],q1_set[1]),
                      np.random.uniform(q2_set[0],q2_set[1]),
                      np.random.uniform(q3_set[0],q3_set[1]),
                      np.random.uniform(q4_set[0],q4_set[1]),
                      np.random.uniform(q5_set[0],q5_set[1])])
        
        if collisionFree(r,q) == True:
            # Find closest vertix in V 
            for idx, node in enumerate(NodeList):
                distance = findDistance(node.q,q)
                if distance < best_distance:
                    best_distance = distance
                    closest_idx = idx
            
            # Now we have q1 and q2
            storeModel,r = steeringFunction(NodeList[closest_idx].q,q)
            for n in range(len(storeModel)):
                if collisionFree(r,storeModel[n])==0:
                    freePath = False
                    break
                freePath = True
            if freePath == True:
                Node_inst = Node(q)
                Node_inst.parent = NodeList[closest_idx]
                NodeList.append(Node_inst)
                
        # if the goal is close to the last added node        
        if (abs(findDistance(NodeList[-1].q,goal)) < 5):
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


# Simulate:
start = np.array([0.0,0.0,0.0,0.0,0.0])
goal = np.array([5.0,5.0,np.pi,0.0,0.0])
NodeList = RRT(start,goal)
for i in range(0,len(NodeList)):
    x,y = NodeList[i].q[0:2]
    plt.plot(x,y)
plt.show()