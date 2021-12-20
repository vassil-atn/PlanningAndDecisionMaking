import numpy as np
from robotModel import Robot
#from shapely.geometry import Polygon,Point
from collision_detection import checkPolysIntersecting, checkPolyCircleIntersecting
import matplotlib.pyplot as plt
#from matplotlib.patches import Arc


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

def steeringFunction(q_init,q_des,plot=False):
    storeModel = []
    r = Robot(mp=np.array([q_init[0],q_init[1]]),phi=q_init[2],q=np.array([q_init[3],q_init[4]]))

    T = 3
    dt = 0.01
    prev_error = 0
    error_i = 0
    
    # Define the direction to move the mobile robot to the desired position
    vector = np.array([q_des[0]-q_init[0],q_des[1]-q_init[1]])/np.linalg.norm([q_des[0]-q_init[0],q_des[1]-q_init[1]])
    
    # PID CONTROLLER:
    Kp = 3
    Ki = 0.01
    Kd = 0.01
    
    # Trajectory part 1 - just rotate the phi to the desired direction of movement
    for i in range(0,int(T/dt)):
        X_des = [r.mp[0],r.mp[1],np.arctan2(vector[1],vector[0]),r.q[0],r.q[1]]
        X = r.phi

        error = X_des[2] - X
        if abs(error)<0.01*np.pi/180:
            break
        error_d = (prev_error - error)
        error_i = error_i + error

        mp_dot = np.zeros(2) # mobile base doesn't move, only rotates
        phi_dot = Kp*error + Ki*error_i*dt + Kd*error_d/dt
        dq = np.zeros(2) # the manipulator arms are kept fixed
        
        u = np.array([-phi_dot*r.h,phi_dot*r.h])
        if abs(u[0])>r.u_limits and u[0]<0:
            u[0] = -r.u_limits
            u[1] = r.u_limits
        if abs(u[0])>r.u_limits and u[1]<0:
            u[0] = r.u_limits
            u[1] = -r.u_limits
        
        phi_dot = (u[1]-u[0])/(2*r.h)
        
        # Integrate numerically
        mp,phi,q,p,theta,p_joint_1 = integrateNumerically(r,mp_dot,phi_dot,dq,dt)
        
        r.Update(mp,phi,p,theta,q,dq,np.array([0.0,0.0]),p_joint_1)
        storeModel.append(np.array([mp[0],mp[1],phi,q[0],q[1]]))
        
        # Save error for the derivative controller
        prev_error = error
        
        if plot==True:
            # Visualize the movement
            visualizeMovement(r)
        

    # Trajectory part 2 - move the mobile robot in straight line
    prev_error = [0,0]
    error_i = [0,0]
    for i in range(0,int(T/dt)):
        X_des = [q_des[0],q_des[1],r.phi,r.q[0],r.q[1]]
        X = np.array([r.mp[0],r.mp[1]])
        
        error = X_des[0:2] - X
        if (np.absolute(error)<0.0001).all():
            break
        error_d = (prev_error - error)
        error_i = error_i + error
        
        mp_dot = Kp*error + Ki*error_i*dt + Kd*error_d/dt
        phi_dot = 0 # mobile base doesn't rotate
        dq = np.zeros(2) # manipulator arms are kept fixed
        
        u = np.array([[np.linalg.norm(mp_dot)],[np.linalg.norm(mp_dot)]])
        if u[0]>r.u_limits:
            u = np.array([[r.u_limits],[r.u_limits]])
            
        mp_dot = np.array([[np.cos(r.phi)/2,np.cos(r.phi)/2],[np.sin(r.phi)/2,np.sin(r.phi)/2]])@u
        mp_dot = np.reshape(mp_dot,2)
        
        # Integrate numerically
        mp,phi,q,p,theta,p_joint_1 = integrateNumerically(r,mp_dot,phi_dot,dq,dt)
        
        r.Update(mp,phi,p,theta,q,dq,np.array([0.0,0.0]),p_joint_1)
        storeModel.append(np.array([mp[0],mp[1],phi,q[0],q[1]]))
        
        # Save error for the derivative controller
        prev_error = error
        
        if plot==True:
            # Visualize the movement
            visualizeMovement(r)

    # Trajectory part 3 - rotate phi and [q1,q2] to the desired values
    prev_error = 0
    error_i = 0    
    for i in range(0,int(T/dt)):
        X_des = [r.mp[0],r.mp[1],q_des[2],q_des[3],q_des[4]]
        X = np.array([r.phi,r.q[0],r.q[1]])
        
        error = X_des[2:] - X
        if (np.absolute(error)<0.01*np.pi/180).all():
            break
        error_d = (prev_error - error)
        error_i = error_i + error
        
        mp_dot = np.zeros(2) # mobile base doesn't move, only rotates
        Q_dot = Kp*error + Ki*error_i*dt + Kd*error_d/dt
        phi_dot = Q_dot[0]
        dq = Q_dot[1:]
        
        u = np.array([-phi_dot*r.h,phi_dot*r.h])
        if abs(u[0])>r.u_limits and u[0]<0:
            u[0] = -r.u_limits
            u[1] = r.u_limits
        if abs(u[0])>r.u_limits and u[1]<0:
            u[0] = r.u_limits
            u[1] = -r.u_limits
        if abs(dq[0])>r.dq_limits and dq[0]>0:
            dq[0] = r.dq_limits
        if abs(dq[0])>r.dq_limits and dq[0]<0:
            dq[0] = -r.dq_limits
        if dq[1]>r.dq_limits and dq[1]>0:
            dq[1] = r.dq_limits
        if dq[1]>r.dq_limits and dq[1]<0:
            dq[1] = -r.dq_limits
        
        phi_dot = (u[1]-u[0])/(2*r.h)
        
        # Integrate numerically
        mp,phi,q,p,theta,p_joint_1 = integrateNumerically(r,mp_dot,phi_dot,dq,dt)
        
        r.Update(mp,phi,p,theta,q,dq,np.array([0.0,0.0]),p_joint_1)
        storeModel.append(np.array([mp[0],mp[1],phi,q[0],q[1]]))
        
        # Save error for the derivative controller
        prev_error = error
        
        if plot==True:
            # Visualize the movement
            visualizeMovement(r)
        
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

def plotConfig(ax, q, collision = False, r = None):
    if collision:
        ax.plot(q[0],q[1],'or')
    else:
        ax.plot(q[0],q[1],'ob')
        
    if r != None:
        if collision:
            ax.add_patch(plt.Circle((q[0], q[1]), r.R, color='r',fill=False))
        else:
            ax.add_patch(plt.Circle((q[0], q[1]), r.R, color='b',fill=False))
        _,_,_,j1,j2 = r.ForwardKinematicsConfig(q)
        ax.plot([q[0],j1[0],j2[0]],[q[1],j1[1],j2[1]],'k')
        
    plt.show()
    plt.pause(0.001)


def plotPath(ax, q1, q2, collision = False):
    if collision:
        ax.plot([q1[0],q2[0]],[q1[1],q2[1]],'-r')
    else:
        ax.plot([q1[0],q2[0]],[q1[1],q2[1]],'-b')
        
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


def RRT(start,goal_end,room_width,room_height,N=100,obstacles=None):
    
    # Init seed for repeatability
    np.random.seed(2)
        
    NodeList = []
    r = Robot()
    
    # Draw room
    fig, ax = plt.subplots()
    ax.set_aspect('equal','box')
    ax.set_xlim([0, room_width])
    ax.set_ylim([0, room_height])
    draw_room(ax, obstacles)
    ax.add_patch(plt.Circle(start[0:2],2.5,color='red',fill=False))
    ax.add_patch(plt.Circle(goal_end[0:2],2.5,color='green',fill=False))
    
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
    plotConfig(ax,goal,False,r)
   
    # First add the starting node:
    Node_inst = Node(start)
    NodeList.append(Node_inst)
    # Define sets for each configuration variable:
    q1_set = np.array([0,room_width])
    q2_set = np.array([0,room_height])
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
        
        if collisionFree(r,q,obstacles) == True:
            print(f'Sample number {i} config is collision free!')
            #plotConfig(ax, q, collision=False, r=r)
            # Find closest vertix in V 
            for idx, node in enumerate(NodeList):
                distance = findDistance(node.q,q)
                if distance < best_distance:
                    best_distance = distance
                    closest_idx = idx
            
            # Now we have q1 and q2
            storeModel,r = steeringFunction(NodeList[closest_idx].q,q)
            for n in range(len(storeModel)):
                if collisionFree(r,storeModel[n],obstacles)==0:                    
                    freePath = False
                    print(f'Sample number {i} trajectory has a collision!')
                    plotPath(ax, q, NodeList[closest_idx].q, collision=True)
                    #plotTrajectory(ax, np.array(storeModel), collision=True, r=r)

                    break
                freePath = True
                
            if freePath == True:
                Node_inst = Node(q)
                Node_inst.parent = NodeList[closest_idx]
                NodeList.append(Node_inst)
                print(f'Sample number {i} added to tree!')
                plotPath(ax, q, NodeList[closest_idx].q, collision=False)
                #plotTrajectory(ax, np.array(storeModel), collision=False, r=r)
        else:
            print(f'Sample number {i} config has a collision!')
            plotConfig(ax, q, collision=True)
                
        # if the goal is close to the last added node        
        #if (abs(findDistance(NodeList[-1].q,goal)) < 5):
        # Define path from last added node to goal:
        storeModel,r = steeringFunction(NodeList[-1].q,goal)
        # Check if path is free:
        for n in range(len(storeModel)):
            if collisionFree(r,storeModel[n],obstacles)==0:
                freePath = False
                break
            freePath = True
            
        if freePath == True:
            print(f'Sample number {i} has a path to goal!')
            #plotTrajectory(ax, np.array(storeModel), collision=False, r=r)
            Node_inst = Node(goal)
            Node_inst.parent = NodeList[-1]
            NodeList.append(Node_inst)
            break

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
        ax.plot([path[n].q[0],path[n+1].q[0]],[path[n].q[1],path[n+1].q[1]],color='green')
        plotConfig(ax, path[n].q,False,r)
    plt.show()
    plt.pause(0.001)

    # New plot for final trajectory
    fig2, ax2 = plt.subplots()
    ax2.set_aspect('equal','box')
    ax2.set_xlim([0, room_width])
    ax2.set_ylim([0, room_height])

    # Animate the final trajectory
    for n in range(len(path)-1):
        traj,r = steeringFunction(path[n].q,path[n+1].q)
        for traj_q in traj:
            plt.cla()
            draw_room(ax2, obstacles)
            plotConfig(ax2, traj_q, collision=False, r=r)

    return NodeList
