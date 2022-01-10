import numpy as np
from robotModel import Robot
from collision_detection import checkPolysIntersecting, checkPolyCircleIntersecting
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
savefigs = False

class Node:
    def __init__(self,q,cost=0):
        self.q = q
        self.parent = None
        self.cost = cost
        self.vis = []
            

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
    distance = np.linalg.norm(q1[0:2] - q2[0:2]) + HaarMeasure(q1[2],q2[2]) + HaarMeasure(q1[3],q2[3]) + HaarMeasure(q1[4],q2[4]) 
    
    return distance


def integrateNumerically(r,mp_dot,phi_dot,dq,dt):
    # Integrate numerically:
    mp = r.mp.copy()
    phi = r.phi
    p = r.p.copy()
    theta = r.theta
    q = r.q.copy()
        
    mp += mp_dot*dt
    phi += phi_dot*dt
    q += dq[:]*dt
    p[0] = mp[0] + np.cos(phi)*(r.l[0]*np.cos(q[0]) + r.l[1]*np.cos(q[0]+q[1])) - np.sin(phi)*(r.l[0]*np.sin(q[0]) + r.l[1]*np.sin(q[0]+q[1]))
    p[1] = mp[1] + np.sin(phi)*(r.l[0]*np.cos(q[0]) + r.l[1]*np.cos(q[0]+q[1])) + np.cos(phi)*(r.l[0]*np.sin(q[0]) + r.l[1]*np.sin(q[0]+q[1]))
    theta = phi + q[0] + q[1]
    
    # Find position of the first joint (to plot)
    
    # Position of the first joint w.r.t the mobile base:
    p_em = np.array([r.l[0]*np.cos(q[0]),
                     r.l[0]*np.sin(q[0])])
    
    p_joint_1 = np.zeros(2)
    p_joint_1[0] = mp[0] + np.cos(phi)*p_em[0] - np.sin(phi)*p_em[1]
    p_joint_1[1] = mp[1] + np.sin(phi)*p_em[0] + np.cos(phi)*p_em[1]
    
    return mp,phi,q,p,theta,p_joint_1


def visualizeMovement(r):
    plt.cla()
    # plt.xlim([-10,10])
    # plt.ylim([-10,10])
    #plt.axis('equal')
    ax = plt.gca()
    ax.set_aspect('equal', adjustable='box')
    # Plot the body of the robot:
    robotBody = plt.Circle((r.mp[0], r.mp[1]), r.R, color='r',fill=False)
    plt.plot([r.mp[0],r.mp[0]+1.5*np.cos(r.phi)],[r.mp[1],r.mp[1]+1.5*np.sin(r.phi)],color='k')
    ax.add_patch(robotBody)
    # Plot link 1:
    plt.plot([r.mp[0],r.p_joint_1[0]],[r.mp[1],r.p_joint_1[1]],color='orange')
    plt.plot(r.p_joint_1[0],r.p_joint_1[1],'.',color='k')
    
    # Plot link 2:
    plt.plot([r.p_joint_1[0],r.p[0]],[r.p_joint_1[1],r.p[1]],color='orange')
    # Add the gripper
    angle = np.rad2deg(r.theta)
    gripper = Arc((r.p[0]+0.25*np.cos(r.theta), r.p[1]+0.25*np.sin(r.theta)),0.5,0.5,angle+90,0,180, color='r')
    ax.add_patch(gripper)
    
    base_box,polygon1,polygon2 = collisionBox(r, np.array([r.mp[0],r.mp[1],r.phi,r.q[0],r.q[1]]))
    base_x,base_y = base_box.exterior.xy
    plt.plot(base_x,base_y,'g')
    x1,y1 = polygon1.exterior.xy
    x2,y2 = polygon2.exterior.xy
    
    poly1 = Polygon([np.array([1,1]),np.array([1,2]),np.array([2,2]),np.array([2,1])])
    poly2 = Polygon([np.array([3,3]),np.array([4,3]),np.array([4,4]),np.array([3,4])])
    poly3 = Polygon([np.array([7,7]),np.array([8,7]),np.array([8,8]),np.array([7,8])])
    
    
    plt.plot(poly1.exterior.xy[0],poly1.exterior.xy[1])
    plt.plot(poly2.exterior.xy[0],poly2.exterior.xy[1])
    plt.plot(poly3.exterior.xy[0],poly3.exterior.xy[1])
            
    plt.plot(x1,y1)
    plt.plot(x2,y2)
    plt.grid()
    plt.pause(0.001)

def steeringFunction(q_init,q_des,plot=False,obstacles=None,phi_desired=True):
    storeModel = []
    r = Robot(mp=np.array([q_init[0],q_init[1]]),phi=q_init[2],q=np.array([q_init[3],q_init[4]]))

    T = 100
    dt = 0.01
    prev_error = 0
    error_i = 0
    
    # Define the direction to move the mobile robot to the desired position
    vector = np.array([q_des[0]-q_init[0],q_des[1]-q_init[1]])/np.linalg.norm([q_des[0]-q_init[0],q_des[1]-q_init[1]])
    
    # Check if the mobile base will collide with any obstacle
    collision_free = True
    if obstacles != None:
        boxpoly = collisionBoxPath(vector,r,q_init,q_des)
        collision_free = True
        for obst in obstacles:
            if checkPolysIntersecting(boxpoly,obst):
                collision_free = False
                break
    
    if collision_free: # if mobile base won't collide (the arms might still collide)
        # PID CONTROLLER:
        Kp = 10
        Ki = 0.5
        Kd = 0.1
    
        firstRun = np.array([True,True,True])
    
        mode = 0
        # Steering function
        for i in range(0,int(T/dt)):
    
            vector = np.array([q_des[0]-r.mp[0],q_des[1]-r.mp[1]])/np.linalg.norm([q_des[0]-r.mp[0],q_des[1]-r.mp[1]])    
            X_des = np.array([q_des[0],q_des[1],np.arctan2(vector[1],vector[0]),q_des[3],q_des[4]])
            X = np.array([r.mp[0],r.mp[1],r.phi,r.q[0],r.q[1]])
            
            if obstacles != None:
                if collisionFree(r,X,obstacles)==0:
                    # There's a collision somewhere along the trajectory
                    storeModel = []
                    break
            
            if np.all(abs(X-q_des) < 0.1) and phi_desired==True: # we care about the phi of q_des (only in the case q_des is the goal config)
                break
            
            if phi_desired==False: # we don't care about the phi of q_des (in case of intermediate nodes)
                q_des_no_phi = np.array([q_des[0],q_des[1],q_des[3],q_des[4]])
                X_no_phi = np.array([r.mp[0],r.mp[1],r.q[0],r.q[1]])
                if np.all(abs(X_des[0:2] - X[0:2]) <= 0.1) and np.all(abs(X_des[3:5] - X[3:5]) <= 0.1):
                    break
                
            
            # First make sure the heading to the configuration is correct
            if abs(X_des[2] - X[2]) > 0.05 and mode != 3:
                if firstRun[0] == True:
                    error_i = 0
                    prev_error = 0
                firstRun[0] = False
                firstRun[1:3] = True
                error = np.zeros([5])
                error[2] = X_des[2] - X[2]
                mode = 1
    
    
                #
            # If the heading is correct check the xy position of the base and the joint angles:
            elif np.any(abs(X_des[0:2] - X[0:2]) > 0.1) or np.any(abs(X_des[3:5] - X[3:5]) > 0.1):
                if firstRun[1] == True:
                    error_i = 0
                    prev_error = 0
                firstRun[1] = False
                firstRun[0] = True
                firstRun[2] = True
                error = np.zeros([5])
                error = X_des - X
                error[2] = 0
                mode = 2
            # Finally ensure that the robot rotates to the desired phi (only if q_des is the goal config)
            elif abs(q_des[2]- X[2])>0.1 and phi_desired==True:
                X_des = np.array([q_des[0],q_des[1],q_des[2],q_des[3],q_des[4]])
                if firstRun[2] == True:
                    error_i = 0
                    prev_error = 0
                firstRun[2] = False
                firstRun[0:2] = True
                error = np.zeros([5])
                error[2] = X_des[2] - X[2]
                mode = 3
            
            
            error_d = (prev_error - error)
            error_i = error_i + error
                
    
            X_dot = Kp*error + Ki*error_i*dt + Kd*error_d/dt
    
    
            mp_dot_des = X_dot[0:2]
    
            phi_dot_des = X_dot[2]
            dq_des = X_dot[3:5]
            
            # If the robot needs to move to target the u_des is different than when it needs to rotate:
            if mode == 1 or mode == 3:
                u_des = np.array([-phi_dot_des*r.h,phi_dot_des*r.h])
            else:
                u_des = np.array([np.linalg.norm(mp_dot_des),np.linalg.norm(mp_dot_des)])
            
            
            u_des = np.clip(u_des,-r.u_limits,r.u_limits)
            dq_des = np.clip(dq_des,-r.dq_limits,r.dq_limits)
            # Simulate the movement:
            phi_dot = (u_des[1]-u_des[0])/(2*r.h)
            mp_dot = ((np.array([[np.cos(r.phi)/2,np.cos(r.phi)/2],[np.sin(r.phi)/2,np.sin(r.phi)/2]])).dot(u_des)).T
            #mp_dot = np.reshape(mp_dot,2)
            dq = dq_des.copy()
            # Integrate numerically
            mp,phi,q,p,theta,p_joint_1 = integrateNumerically(r,mp_dot,phi_dot,dq,dt)
            
            r.Update(mp,phi,p,theta,q,dq,np.array([0.0,0.0]),p_joint_1) 
    
            # Save error for the derivative controller
            prev_error = error     
            storeModel.append(np.array([mp[0],mp[1],phi,q[0],q[1]]))
            
            # Save error for the derivative controller
            prev_error = error
        
# =============================================================================
#         if plot==True:
#             # Visualize the movement
#             visualizeMovement(r)
# =============================================================================
        
    return storeModel,r


def collisionFree(r,q,obstacles=None):

    # Build the bounding boxes for the base and the links:
    poly_link1,poly_link2 = collisionBox(r,q)

    collision_free = True
    if obstacles != None:
        for obst in obstacles:
            if checkPolyCircleIntersecting(obst, q[0], q[1], r.R):
                collision_free = False
                break
            if checkPolysIntersecting(poly_link1,obst): 
                collision_free = False
                break
            if checkPolysIntersecting(poly_link2,obst): 
                collision_free = False
                break

    return collision_free


# This function finds the collision polygon for the mobile base path
def collisionBoxPath(vector,r,q_init,q_des):
    #compute orthogonal vector to vector
    vector_orth = np.array([vector[1],-vector[0]])
    #compute distance between q_init and q_des
    distance = np.linalg.norm([q_des[0]-q_init[0],q_des[1]-q_init[1]])
    c1 = r.mp+r.R*vector_orth
    c2 = r.mp-r.R*vector_orth
    c3 = c2 + distance*vector
    c4 = c1 + distance*vector
    box_poly = np.vstack((c1,c2,c3,c4))
    return box_poly
    

# This function finds the collision polygons for the robot links only
def collisionBox(r,q): 
    mp,p_centre1,p_centre2,joint1,joint2 = r.ForwardKinematicsConfig(q)
    l1_orth = q[2]+q[3]+np.pi/2
    l2_orth = q[2]+q[3]+q[4]+np.pi/2
    thk = 0.1   # half the thickness of the link
    
    # link 1 corners
    c1 = np.array([mp[0]+thk*np.cos(l1_orth), mp[1]+thk*np.sin(l1_orth)])
    c2 = np.array([joint1[0]+thk*np.cos(l1_orth), joint1[1]+thk*np.sin(l1_orth)])
    c3 = np.array([joint1[0]-thk*np.cos(l1_orth), joint1[1]-thk*np.sin(l1_orth)])
    c4 = np.array([mp[0]-thk*np.cos(l1_orth), mp[1]-thk*np.sin(l1_orth)])
    l1_poly = np.vstack((c1,c2,c3,c4))
    # link 2 corners
    c1 = np.array([joint1[0]+thk*np.cos(l2_orth), joint1[1]+thk*np.sin(l2_orth)])
    c2 = np.array([joint2[0]+thk*np.cos(l2_orth), joint2[1]+thk*np.sin(l2_orth)])
    c3 = np.array([joint2[0]-thk*np.cos(l2_orth), joint2[1]-thk*np.sin(l2_orth)])
    c4 = np.array([joint1[0]-thk*np.cos(l2_orth), joint1[1]-thk*np.sin(l2_orth)])
    l2_poly = np.vstack((c1,c2,c3,c4))
    
    return l1_poly,l2_poly


def plotConfig(ax, q, Node, collision = False, r = None, show = True):
    # Mobile base position
    if collision:
        Node.vis.append(ax.plot(q[0],q[1],'or')[0])
    else:
        Node.vis.append(ax.plot(q[0],q[1],'ob')[0])
        
    # Full configuration       
    if r != None:
        # Heading
        Node.vis.append(ax.plot([q[0],q[0]+0.75*np.cos(q[2])],[q[1],q[1]+0.5*np.sin(q[2])],color='k')[0])
        # Mobile base
        if collision:
             Node.vis.append(ax.add_patch(plt.Circle((q[0], q[1]), r.R, color='r',fill=False)))
        else:
             Node.vis.append(ax.add_patch(plt.Circle((q[0], q[1]), r.R, color='b',fill=False)))
        # Arms
        mp,pc1,pc2,j1,j2 = r.ForwardKinematicsConfig(q)
        Node.vis.append(ax.plot([q[0],j1[0],j2[0]],[q[1],j1[1],j2[1]],'k')[0])
        theta = np.sum(q[2:5])
        angle = np.rad2deg(theta)
        p = j2
        gripper = Arc((p[0]+0.25*np.cos(theta), p[1]+0.25*np.sin(theta)),0.5,0.5,angle+90,0,180, color='r')
        Node.vis.append(ax.add_patch(gripper))
    if show:    
        plt.show()
    plt.pause(0.000001)


def plotPath(ax, q1, q2, Node2, collision = False,show=True):
    if collision:
        Node2.vis.append(ax.plot([q1[0],q2[0]],[q1[1],q2[1]],'-r')[0])
    else:
        Node2.vis.append(ax.plot([q1[0],q2[0]],[q1[1],q2[1]],'-b')[0])
    if show:    
        plt.show()
    plt.pause(0.001)


def plotTrajectory(ax, traj, collision=False, r = None):
    if collision:
        ax.plot(traj[-1,0],traj[-1,1],'xm')
        ax.plot(traj[:,0],traj[:,1],'--m')
    else:
        ax.plot(traj[-1,0],traj[-1,1],'xc')
        ax.plot(traj[:,0],traj[:,1],'--c')
    
    if r != None:
        for traj_q in traj:
            plotConfig(ax, traj_q, collision=collision, r=r)
    
    plt.show()
    plt.pause(0.001)


def draw_room(win, obstacles):
    for obst in obstacles:
        obs = plt.Polygon(obst, color='black', fill=True)
        win.add_patch(obs)
    return

def clearVisNode(Node_inst):
    plt.pause(0.1)
    try:
        Node_inst.vis.remove()
    except:
        for el in Node_inst.vis:
            el.remove()
            
    Node_inst.vis = []

def RRT(start,goal_end,room_width,room_height,N=100,obstacles=None):
    
    # Init seed for repeatability
    np.random.seed(2)
        
    NodeList = []
    goalNode_index = None
    r = Robot()
    
    # Draw room
    fig, ax = plt.subplots()
    ax.set_aspect('equal','box')
    ax.set_xlim([0, room_width])
    ax.set_ylim([0, room_height])
    draw_room(ax, obstacles)
    ax.add_patch(plt.Circle(start[0:2],2.5,color='red',fill=False))
    ax.add_patch(plt.Circle(goal_end[0:2],2.5,color='green',fill=False))
    
    # First add the starting node:
    Node_inst = Node(start)
    NodeList.append(Node_inst)
    
    # Define goal configuration based on end effector target TODO - check for collision
    # Robot is redundant so 2dof can be selected randomly
    j1_g = np.random.uniform(0,2*np.pi)
    j2_g = np.random.uniform(0,2*np.pi)
    # Heading set to ensure correct endpoint orientation
    phi_g = goal_end[2]-j1_g-j2_g
    # Base position calculation
    px_g = r.l[0]*np.cos(j1_g)+r.l[1]*np.cos(j1_g+j2_g)
    py_g = r.l[0]*np.sin(j1_g)+r.l[1]*np.sin(j1_g+j2_g)
    mp_g = np.array([goal_end[0],goal_end[1]])-r.rotationMatrix(phi_g)@np.array([px_g,py_g])
    goal = np.array([mp_g[0],mp_g[1],phi_g,j1_g,j2_g])
    plotConfig(ax,goal,Node_inst, False,r)
    
    for i in range(0,N):

        print(f'Running sample number {i}')
        best_distance = float('inf')
        # Pick a random point in C-space
        q = np.array([np.random.uniform(0,room_width),
                      np.random.uniform(0,room_height),
                      np.random.uniform(0,2*np.pi),
                      np.random.uniform(0,2*np.pi),
                      np.random.uniform(0,2*np.pi)])
        
        if collisionFree(r,q,obstacles) == True:
            print(f'Sample number {i} config is collision free!')

            Node_inst = Node(q)
            plotConfig(ax, q, Node_inst,collision=False)
            # Find closest vertix in V 
            for idx, node in enumerate(NodeList):
                distance = findDistance(node.q,q)
                if distance < best_distance:
                    best_distance = distance
                    closest_idx = idx
            
            # Now we have q1 and q2
            storeModel,r = steeringFunction(NodeList[closest_idx].q,q,plot=False,obstacles=obstacles)
            freePath = True
            if not(storeModel): #storeModel empty due to some collision
                freePath = False
                print(f'Sample number {i} trajectory has a collision!')
                
            # for n in range(len(storeModel)):
            #     if collisionFree(r,storeModel[n],obstacles)==0:                    
            #         freePath = False
            #         print(f'Sample number {i} trajectory has a collision!')
            #         plotPath(ax, q, NodeList[closest_idx].q, collision=True)
            #         #plotTrajectory(ax, np.array(storeModel), collision=True, r=r)

            #         break
            #     freePath = True
                
            if freePath == True:
                Node_inst = Node(q)
                Node_inst.parent = NodeList[closest_idx]
                NodeList.append(Node_inst)
                print(f'Sample number {i} added to tree!')
                plotPath(ax, NodeList[closest_idx].q,q, Node_inst, collision=False)
                #plotTrajectory(ax, np.array(storeModel), collision=False, r=r)
        else:
            Node_blocked = Node(q)
            print(f'Sample number {i} config has a collision!')
            plotConfig(ax,Node_blocked.q,Node_blocked, collision=True)
            clearVisNode(Node_blocked)
            
                
        # if the goal is close to the last added node        
        #if (abs(findDistance(NodeList[-1].q,goal)) < 5):
        # Define path from last added node to goal:
        storeModel,r = steeringFunction(NodeList[-1].q,goal,plot=False,obstacles=obstacles)
        # Check if path is free:
        freePath = True
        if not(storeModel): #storeModel empty due to some collision
            freePath = False
            
        # for n in range(len(storeModel)):
        #     if collisionFree(r,storeModel[n],obstacles)==0:
        #         freePath = False
        #         break
        #     freePath = True
            
        if freePath == True:
            print(f'Sample number {i} has a path to goal!')
            #plotTrajectory(ax, np.array(storeModel), collision=False, r=r)
            Node_inst = Node(goal)
            Node_inst.parent = NodeList[-1]
            NodeList.append(Node_inst)
            goalNode_index = len(NodeList)-1
            break
    
    if goalNode_index == None:
        print('No path to goal found')
    else:
        print('Path to goal found')
        # Get path from tree
        currentNode = NodeList[-1]
        path = []
        path.append(NodeList[-1])
        while currentNode != NodeList[0]:
            path.append(currentNode.parent)
            currentNode = currentNode.parent
        path= path[::-1]
        
        # Draw final path and intermediate configs
        for n in range(len(path)-1):
            ax.plot([path[n].q[0],path[n+1].q[0]],[path[n].q[1],path[n+1].q[1]],color='green')[0]
            plotConfig(ax, path[n].q,path[n],False,r)
        plotConfig(ax, path[-1].q, path[-1],False,r)
        plt.show()
        plt.pause(0.001)
    
        # New plot for final trajectory
        fig2, ax2 = plt.subplots()
        ax2.set_aspect('equal','box')
        ax2.set_xlim([0, room_width])
        ax2.set_ylim([0, room_height])
    
        # Animate the final trajectory
        for n in range(len(path)-1):
            if n==0:
                traj,r = steeringFunction(path[n].q,path[n+1].q,plot=False,obstacles=None,phi_desired=False)
                last_config = traj[-1]
            elif n>0 and n<len(path)-1-1:
                traj,r = steeringFunction(last_config,path[n+1].q,plot=False,obstacles=None,phi_desired=False)
                last_config = traj[-1]
            else: # n==len(path)-1-1: # if n corresponds to the last iteration (from penultimate node to goal node)
                traj,r = steeringFunction(last_config,path[n+1].q,plot=False,obstacles=None,phi_desired=True)
            for traj_q in traj:
                plt.cla()
                draw_room(ax2, obstacles)
                plotConfig(ax2, traj_q,Node(traj_q), collision=False, r=r)

    return NodeList


# Function to check if a trajectory between two configurations has any collisions
def pathCollisionFree(q1, q2, obstacles):
    storeModel,r = steeringFunction(q1,q2)
    for n in range(len(storeModel)):
        if collisionFree(r,storeModel[n],obstacles)==0:
            return False
    return True


# This function finds the nearest nodes around the new node in a given radius
def findNearestNodes(q,NodeList):
    nearest_nodes = []
    # Define the radius in which to check for more optimal paths 
    radius = 20 # TODO - change to minimum needed for optimal?
    for idx,node in enumerate(NodeList):
        if findDistance(node.q,q) < radius:
            nearest_nodes.append(idx)
    return nearest_nodes


# This function finds the lowest cost node in the vicinity of the new node
# to be selected as a candidate for its parent node
def chooseParent(NodeList,nearest_nodes,q):
    min_cost = float('inf')
    for node in nearest_nodes:
        cost = NodeList[node].cost + findDistance(NodeList[node].q,q)
        if  cost < min_cost:
            min_cost = cost
            parent_node = NodeList[node]
            parent_index = node
    return parent_node,parent_index


# This function updates the costs of leaves once their parent node's cost has been updated
def changeLeavesCost(rewired_node,NodeList): 
    for node in NodeList:
        if node.parent == rewired_node:
            print("CHANGING LEAF COST")
            node.cost = rewired_node.cost + findDistance(rewired_node.q,node.q)
            changeLeavesCost(node,NodeList)
        

def RRT_star(start,goal_end,room_width,room_height,N=100,obstacles=None):
    
    # Init seed for repeatability
    np.random.seed(2)
    
    NodeList = []
    goalNode_index = None
    r = Robot()

    # Draw room
    n_treefig = 0
    fig, ax = plt.subplots(figsize=(9,6.75))
    ax.set_aspect('equal','box')
    #ax.set_xlim([0, room_width])
    #ax.set_ylim([0, room_height])
    draw_room(ax, obstacles)

    ax.add_patch(plt.Circle(start[0:2],2.5,color='red',fill=False))
    ax.add_patch(plt.Circle(goal_end[0:2],2.5,color='green',fill=False))
    ax.add_patch(plt.Circle(goal_end[0:2],0.2,color='green'))
    ax.plot([goal_end[0],goal_end[0]+np.cos(goal_end[2])],[goal_end[1],goal_end[1]+np.sin(goal_end[2])],'-g')
    plt.show()
    
    # Define goal configuration based on end effector target 
    # TODO - check for collision (remove requirement that obstacles aren't created 2m around goal)
    # Robot is redundant so 2dof can be selected randomly
    j1_g = np.random.uniform(0,2*np.pi)
    j2_g = np.random.uniform(0,2*np.pi)
    # Heading set to ensure correct endpoint orientation
    phi_g = goal_end[2]-j1_g-j2_g
    # Base position calculation
    px_g = r.l[0]*np.cos(j1_g)+r.l[1]*np.cos(j1_g+j2_g)
    py_g = r.l[0]*np.sin(j1_g)+r.l[1]*np.sin(j1_g+j2_g)
    mp_g = np.array([goal_end[0],goal_end[1]])-r.rotationMatrix(phi_g)@np.array([px_g,py_g])
    goal = np.array([mp_g[0],mp_g[1],phi_g,j1_g,j2_g])
    
   
    # First add the starting node:
    Node_inst = Node(start)
    NodeList.append(Node_inst)
    
    for i in range(0,N):

        print(f'Running sample number {i}')
        best_distance = float('inf')
        # Pick a random point in C-space
        q = np.array([np.random.uniform(0,room_width),
                      np.random.uniform(0,room_height),
                      np.random.uniform(0,2*np.pi),
                      np.random.uniform(0,2*np.pi),
                      np.random.uniform(0,2*np.pi)])
        
        if collisionFree(r,q,obstacles) == True:
            print(f'Sample number {i} config is collision free!')
            Node_inst = Node(q)
            plotConfig(ax, q, Node_inst,collision=False)
            
            nearest_nodes = findNearestNodes(q,NodeList)
            if goalNode_index in nearest_nodes:
                nearest_nodes.remove(goalNode_index)
                
                
            # Find closest vertix in V 
            for idx, node in enumerate(NodeList):
                distance = findDistance(node.q,q)
                if distance < best_distance:
                    best_distance = distance
                    closest_idx = idx
            
            # Now we have q1 and q2
            storeModel,r = steeringFunction(NodeList[closest_idx].q,q,plot=False,obstacles=obstacles)
            freePath = True
            if not(storeModel): #storeModel empty due to some collision
                freePath = False
                clearVisNode(Node_inst)
                print(f'Sample number {i} trajectory has a collision!')
                   
                            
            if freePath == True:
                Node_inst.parent = NodeList[closest_idx]
                Node_inst.cost = NodeList[closest_idx].cost + best_distance
                #NodeList.append(Node_inst)
                print(f'Sample number {i} added to tree!')
                plotPath(ax, q, NodeList[closest_idx].q, Node_inst, collision=False)
                #plotTrajectory(ax, np.array(storeModel), collision=False, r=r)
                
                # Iterate over all the nearest nodes until the best parent node with a valid path is found
                # or the list runs out of elements. Each iteration finds the best parent node and if its path
                # isnt collision free it gets removed.
                for num_nodes_near in range(len(nearest_nodes)):
                    # Choose the best candidate for the parent node:
                    parent_node,parent_index = chooseParent(NodeList,nearest_nodes,q)
                    # Simulate movement to the candidate node and check for collisions
                    storeModel,r = steeringFunction(parent_node.q,q,plot=False,obstacles=obstacles)
                    freePath = True
                    if not(storeModel):
                        freePath = False
                        print(f'Sample number {i} trajectory has a collision!')
                    
                    if freePath == True:
                        if closest_idx != parent_index:
                            Node_inst.parent = parent_node
                            Node_inst.cost = Node_inst.parent.cost + findDistance(Node_inst.parent.q,q)
                            clearVisNode(Node_inst)
                      #      NodeList.append(Node_inst)
                      
                            print('Path has been rewired!')
                            plotConfig(ax, q, Node_inst,collision=False)
                            plotPath(ax, q,Node_inst.parent.q, Node_inst,collision=False)
                        break
                    else:
                        nearest_nodes.remove(parent_index) # remove the best candidate from the nearest_nodes
                        # and check the next one
                if savefigs:
                    plt.savefig("figs/tree/"+str(n_treefig))
                    n_treefig += 1        
                NodeList.append(Node_inst)        
                # Check if any of the nearby nodes need to be updated due to the new node:
                # Reuse nearest_nodes since the only ones removed had collision to the new node
                for nearest_node in nearest_nodes:
                    dist = findDistance(NodeList[-1].q,NodeList[nearest_node].q) # distance between new node and each nearest_node
                    if  (dist + NodeList[-1].cost) < NodeList[nearest_node].cost:
                        # Simulate movement to the candidate node and check for collisions
                        storeModel,r = steeringFunction(NodeList[-1].q,NodeList[nearest_node].q,plot=False,obstacles=obstacles)
                        freePath = True
                        if not(storeModel):
                            freePath = False
                            
                        if freePath == True:
                            print("REWIRING NODE")
                            # If the cost through the new node to some nearest node is lower than that node's
                            # original cost - update it
                            NodeList[nearest_node].parent = NodeList[-1]
                            NodeList[nearest_node].cost = dist + NodeList[-1].cost
                            # Clear the plot for the previous path and add the new path
                            clearVisNode(NodeList[nearest_node])
                            plotConfig(ax, NodeList[nearest_node].q, NodeList[nearest_node],collision=False)
                            plotPath(ax, NodeList[nearest_node].q,NodeList[nearest_node].parent.q, NodeList[nearest_node],collision=False,show=False)
                            # Recursively change the cost of each leaf of the reconnected node
                            changeLeavesCost(NodeList[nearest_node],NodeList)
                            
                            plt.show()
                            if savefigs:
                                plt.savefig("figs/tree/"+str(n_treefig))
                                n_treefig += 1
                            
                # CHECK THE GOAL CONDITION:
                
                # if the goal is close to the last added node        
                #if (abs(findDistance(NodeList[-1].q,goal)) < 5):
                # Define path from last added node to goal:
                storeModel,r = steeringFunction(NodeList[-1].q,goal,plot=False,obstacles=obstacles)
                # Check if path is free:
                freePath = True
                if not(storeModel):
                    freePath = False
                            
        else:
            freePath = False
            Node_inst = Node(q)
            print(f'Sample number {i} config has a collision!')
            plotConfig(ax,Node_inst.q,Node_inst, collision=True)
            clearVisNode(Node_inst)  

                  
        if freePath == True:
            print(f'Sample number {i} has a path to goal!')
            # If it's the first time we reached the goal, connect it to the new node
            dist = findDistance(NodeList[-1].q, goal)
            if goalNode_index == None:       
                #plotTrajectory(ax, np.array(storeModel), collision=False, r=r)
                Node_inst = Node(goal)
                Node_inst.parent = NodeList[-1]
                Node_inst.cost = dist + Node_inst.parent.cost
                plotConfig(ax, Node_inst.q, Node_inst,collision=False,r=r)
                plotPath(ax, NodeList[-1].q, Node_inst.q, Node_inst, collision = False)
                plt.show()
                NodeList.append(Node_inst)
                goalNode_index = len(NodeList)-1
            # If goal reached before, change the parent to new node if new path cost is lower
            else:
                if (dist + NodeList[-1].cost) < (NodeList[goalNode_index].cost):
                    NodeList[goalNode_index].parent = NodeList[-1]
                    NodeList[goalNode_index].cost = dist + NodeList[-1].cost
                    plotPath(ax, NodeList[-1].q, NodeList[goalNode_index].q, NodeList[goalNode_index], collision = False)

    if goalNode_index == None:
        print('No path to goal found')
    else:
        print('Path to goal found')
        # Get path from tree
        currentNode = NodeList[goalNode_index]
        path = []
        path.append(currentNode)
        while currentNode != NodeList[0]:
            path.append(currentNode.parent)
            currentNode = currentNode.parent
        path= path[::-1]
        
        # Draw final path and intermediate configs
        for n in range(len(path)-1):
            ax.plot([path[n].q[0],path[n+1].q[0]],[path[n].q[1],path[n+1].q[1]],color='green',lw=3)
            plotConfig(ax, path[n].q,path[n],False,r)
        plotConfig(ax, path[-1].q,path[-1],False,r)
        plt.show()
        plt.pause(0.001)
        if savefigs:
            plt.savefig("figs/tree/"+str(n_treefig))
            n_treefig += 1
    
        # New plot for final trajectory
        fig2, ax2 = plt.subplots(figsize=(9,6.75))
        ax2.set_aspect('equal','box')
        ax2.set_xlim([0, room_width])
        ax2.set_ylim([0, room_height])
    
        # Animate the final trajectory
        n_animfig = 0
        for n in range(len(path)-1):
            if n==0:
                traj,r = steeringFunction(path[n].q,path[n+1].q,plot=False,obstacles=None,phi_desired=False)
                last_config = traj[-1]
            elif n>0 and n<len(path)-1-1:
                traj,r = steeringFunction(last_config,path[n+1].q,plot=False,obstacles=None,phi_desired=False)
                last_config = traj[-1]
            else: # n==len(path)-1-1: # if n corresponds to the last iteration (from penultimate node to goal node)
                traj,r = steeringFunction(last_config,path[n+1].q,plot=False,obstacles=None,phi_desired=True)
            for traj_q in traj:
                plt.cla()
                draw_room(ax2, obstacles)
                plotConfig(ax2, traj_q, Node(traj_q),collision=False, r=r)
                if savefigs:
                    n_animfig += 1
                    if(n_animfig%5 == 0):
                        plt.savefig("figs/anim/"+str(n_animfig))
                    

    return NodeList
