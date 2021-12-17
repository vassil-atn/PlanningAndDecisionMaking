import numpy as np
from robotModel import Robot
#from shapely.geometry import Polygon,Point
from collision_detection import checkPolysIntersecting, checkPolyCircleIntersecting
import matplotlib.pyplot as plt
from matplotlib.patches import Arc


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


def steeringFunction(q1,q2,plot=False):
    # convert q2 to workspace coordinates (x,y,theta), feed into IK model to find the path
    # MODIFY 
    storeModel = []
    #mp,p_centre1,p_centre2,joint_1,joint_2 = r.ForwardKinematicsConfig(q1)
    r = Robot(mp=np.array([q1[0],q1[1]]),phi=q1[2],q=np.array([q1[3],q1[4]]))
    
    mp,p_centre1,p_centre2,joint_1,joint_2 = r.ForwardKinematicsConfig(q2)
    
    
    
    X_des = np.array([joint_2[0],joint_2[1],np.sum(q2[2:5])])
    T = 3
    dt = 0.01
    prev_error = 0
    error_i = 0

    for i in range(0,int(T/dt)):
    
        # PID CONTROLLER:
        Kp = 3
        Ki = 0.01
        Kd = 0.01
        
        lambdaD = 0.001
        # Inverse Dynamics:
    
        J = r.Jacobian(r.u)    
        # Pseudo inverse singularity robust:

        J_inv = J.T.dot(np.linalg.inv(J.dot(J.T)+lambdaD*np.eye(3)))
        
        
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

        
        # Simulate forward motion with these desired commands:
        phi_dot = (u[1] - u[0])/(2*r.h)
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
 
# =============================================================================
#         if plot == True:
#             
#             # VISUALISE THE MOVEMENT
#            
#             plt.cla()
#             #plt.axis('equal')
#             ax = plt.gca()
#             # Plot the body of the robot:
#             robotBody = plt.Circle((mp[0], mp[1]), r.R, color='r',fill=False)
#             plt.plot([mp[0],mp[0]+0.8*np.cos(phi)],[mp[1],mp[1]+0.8*np.sin(phi)])
#             ax.add_patch(robotBody)
#             # Plot link 1:
#             plt.plot([mp[0],p_joint_1[0]],[mp[1],p_joint_1[1]],color='orange')
#             plt.plot(p_joint_1[0],p_joint_1[1],'.',color='k')
#             #
#             # Plot link 2:
#             #
#         # =============================================================================
#         #     p_joint_2 = np.array([0,0])
#         #     p_joint_2[0] = p_joint_1[0] + r.l[1]*np.cos(r.q[0]+r.q[1])
#         #     p_joint_2[1] = p_joint_1[1] + r.l[1]*np.sin(r.q[0]+r.q[1])
#         #     plt.plot([p_joint_1[0],p_joint_2[0]],[p_joint_1[1],p_joint_2[1]],color='green')
#         # =============================================================================
#             plt.plot([p_joint_1[0],p[0]],[p_joint_1[1],p[1]],color='orange')
#             # Add the gripper
#             angle = np.rad2deg(theta)
#             gripper = Arc((p[0]+0.25*np.cos(theta), p[1]+0.25*np.sin(theta)),0.5,0.5,angle+90,0,180, color='r')
#             ax.add_patch(gripper)
#             #
#             
#             base_box,polygon1,polygon2 = collisionBox(r, np.array([mp[0],mp[1],phi,q[0],q[1]]))
#             base_x,base_y = base_box.exterior.xy
#             plt.plot(base_x,base_y,'g')
#             x1,y1 = polygon1.exterior.xy
#             x2,y2 = polygon2.exterior.xy
#             
#             poly1 = Polygon([np.array([1,1]),np.array([1,2]),np.array([2,2]),np.array([2,1])])
#             poly2 = Polygon([np.array([3,3]),np.array([4,3]),np.array([4,4]),np.array([3,4])])
#             poly3 = Polygon([np.array([7,7]),np.array([8,7]),np.array([8,8]),np.array([7,8])])
#     
#             
#             plt.plot(poly1.exterior.xy[0],poly1.exterior.xy[1])
#             plt.plot(poly2.exterior.xy[0],poly2.exterior.xy[1])
#             plt.plot(poly3.exterior.xy[0],poly3.exterior.xy[1])
#                     
#             plt.plot(x1,y1)
#             plt.plot(x2,y2)
#             plt.grid()
#             plt.pause(0.00001)
#             plt.show()
# =============================================================================
        
        if np.all(abs(np.array([r.p[0],r.p[1],r.theta]) - X_des) < 0.01):
            break
        
    return storeModel,r


def collisionFree(r,q,obstacles=None):

    # Build the bounding boxes for the base and the links:
    poly_link1,poly_link2 = collisionBox(r,q)
    
# =============================================================================
#     # Test obstacles for Shapely version
#     # Check if in collision:
#     obstacles = [Polygon([np.array([1,1]),np.array([1,2]),np.array([2,1]),np.array([2,2])]),
#                  Polygon([np.array([3,3]),np.array([4,3]),np.array([4,4]),np.array([3,4])]),
#                  Polygon([np.array([7,7]),np.array([8,7]),np.array([8,8]),np.array([7,8])])]  
# =============================================================================

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
# =============================================================================
#             # For Shapely version
#             if poly_base.intersects(obst) or poly_link1.intersects(obst) or poly_link2.intersects(obst):
#                 collision_free = False
#                 break
# =============================================================================
    return collision_free


# =============================================================================
# # Helper function to define the vertices of the polygons around the robot arm
# def buildPolygons(r,link_centre,angle): 
#     poly = []
#     link_length = r.l[0]
#     link_width = 0.2
#     r = Robot()
#     R = r.rotationMatrix(angle)
#     poly.append(link_centre + R.dot(np.array([-link_length/2,link_width/2])))
#     poly.append(link_centre + R.dot(np.array([link_length/2,link_width/2])))
#     poly.append(link_centre + R.dot(np.array([link_length/2,-link_width/2])))
#     poly.append(link_centre + R.dot(np.array([-link_length/2,-link_width/2])))
#     return poly
# =============================================================================


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
# =============================================================================
# # Shapely version
# # This function finds the collision polygons for the robot (base+arms)
# def collisionBox(r,q): 
#     mp,p_centre1,p_centre2,_,_ = r.ForwardKinematicsConfig(q)
#     phi = q[2]
#     # for the base it's a circle:
#     base_box = Point(mp).buffer(0.6) # Create a circle centered at the centre of the mobile base with radius 0.5
#     polygon1 = Polygon(buildPolygons(r,p_centre1,r.q[0]+phi))
#     polygon2 = Polygon(buildPolygons(r,p_centre2,q[3]+q[4]+phi))
#     
#     return base_box,polygon1,polygon2
# 
# =============================================================================


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


def RRT(start,goal,width,height,ax,N=100,obstacles=None):
    
    # Init seed for repeatability
    np.random.seed(1)
        
    NodeList = []
    r = Robot()
    
    # First add the starting node:
    Node_inst = Node(start)
    NodeList.append(Node_inst)
    # Define sets for each configuration variable:
    q1_set = np.array([0,width])
    q2_set = np.array([0,height])
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
                    #plotPath(ax, q, NodeList[closest_idx].q, collision=True)
                    #plotTrajectory(ax, np.array(storeModel), collision=True)

                    break
                freePath = True
                
            if freePath == True:
                Node_inst = Node(q)
                Node_inst.parent = NodeList[closest_idx]
                NodeList.append(Node_inst)
                print(f'Sample number {i} added to tree!')
                plotPath(ax, q, NodeList[closest_idx].q, collision=False)
                #plotTrajectory(ax, np.array(storeModel), collision=False)
        else:
            print(f'Sample number {i} config has a collision!')
            #plotConfig(ax, q, collision=True)
                
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
            #plotTrajectory(ax, np.array(storeModel), collision=False)
            Node_inst = Node(goal)
            Node_inst.parent = NodeList[-1]
            NodeList.append(Node_inst)
            break

    return NodeList

# =============================================================================
# 
# # Simulate:
# start = np.array([0.0,0.0,0.0,0.0,0.0])
# goal = np.array([5.0,5.0,np.pi,0.0,0.0])
# NodeList = RRT(start,goal)
# currentNode = NodeList[-1]
# path = []
# path.append(NodeList[-1])
# x = []
# y = []
# while currentNode != NodeList[0]:
#     path.append(currentNode.parent)
#     currentNode = currentNode.parent
# path= path[::-1]
# 
# 
# # obstacle (just for plotting)
# 
# for i in range(1,len(path)):
#     steeringFunction(path[i-1].q, path[i].q,plot=True)
#     
#     poly1 = Polygon([np.array([1,1]),np.array([1,2]),np.array([2,2]),np.array([2,1])])
#     poly2 = Polygon([np.array([3,3]),np.array([4,3]),np.array([4,4]),np.array([3,4])])
#     poly3 = Polygon([np.array([7,7]),np.array([8,7]),np.array([8,8]),np.array([7,8])])
#     
#     
#     plt.plot(poly1.exterior.xy[0],poly1.exterior.xy[1])
#     plt.plot(poly2.exterior.xy[0],poly2.exterior.xy[1])
#     plt.plot(poly3.exterior.xy[0],poly3.exterior.xy[1])
#     # Plot positions of the mobile base along the path:
#     plt.plot(path[i-1].q[0],path[i-1].q[1],'x')
#     plt.show()
# 
# =============================================================================
