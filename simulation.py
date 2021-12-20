import numpy as np
from robotModel import Robot
import matplotlib.pyplot as plt
from matplotlib.patches import Arc
#from RRT_algorithm import collisionBox
from shapely.geometry import Polygon,Point


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


def collisionBox(r,q): 
    mp,p_centre1,p_centre2,_,_ = r.ForwardKinematicsConfig(q)
    phi = q[2]
    # for the base it's a circle:
    base_box = Point(mp).buffer(0.6) # Create a circle centered at the centre of the mobile base with radius 0.5
    polygon1 = Polygon(buildPolygons(r,p_centre1,r.q[0]+phi))
    polygon2 = Polygon(buildPolygons(r,p_centre2,q[3]+q[4]+phi))
    
    return base_box,polygon1,polygon2



# Total time in seconds:
T = 3
dt = 0.001 # time step

plt.figure(1)

# Initialise an instance of the robot class
r = Robot()
storeP = np.array([r.p])
# Define desired pose to reach
X_des = np.array([-4,-2,0])
# Initialise some variables:
error_i = 0
prev_error = 0
phi_dot = 0


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
    #J_inv = J.T.dot(np.linalg.inv(J.dot(J.T)))
    X = np.array([r.p[0],r.p[1],r.theta])
    
    error = X_des - X
    error_d = (prev_error - error)
    error_i = error_i + error
    
    # Basically x_dot = K*(x_des - x) (plus the integral and derivative terms)
    X_dot_controller = Kp*error + Ki*error_i*dt + Kd*error_d/dt
    
    Q_dot_des = np.dot(J_inv,X_dot_controller)
    
    # Get the desired inputs for the joints and wheels:
    u = np.clip(Q_dot_des[0:2],a_min = -r.u_limits, a_max = r.u_limits)
    dq = np.clip(Q_dot_des[2:4],a_min = -r.dq_limits, a_max = r.dq_limits)
# =============================================================================
#     u = Q_dot_des[0:2]
#     dq = Q_dot_des[2:4]
# =============================================================================
    
    phi_dot = (u[1] - u[0])/(2*r.h)
    mp_dot = np.array([np.cos(r.phi)*np.sum(u[:])/2, np.sin(r.phi)*np.sum(u[:])/2])
    
    # Simulate forward motion with these desired commands:
    #X_dot = r.ForwardKinematics(u,dq,phi_dot)
        
    #p_dot = X_dot[0:2]
    #theta_dot = X_dot[2]
    
    

# =============================================================================
#     # Q_dot is the input (joints and wheels) states
#     Q_dot = np.array([r.u[0],r.u[1],dq[i,0],dq[i,1]])
#     
#     
#     X_dot = J * Q_dot
#     # X_dot is the velocity and angular velocity of the orientation of the end effector w.r.t the world frame:
# 
#     X_dot = np.dot(J,Q_dot)
#     p_dot = X_dot[0:2]
#     theta_dot = X_dot[2]
# =============================================================================
    

    
## SIMULATE MOVEMENT
    # Simulate forward motion with these desired commands:
    phi_dot = (u[1] - u[0])/(2*r.h)
    X_dot = r.ForwardKinematics(u,dq,phi_dot)
        
    p_dot = X_dot[0:2]
    theta_dot = X_dot[2]
    
    mp_dot = np.array([np.cos(r.phi)*np.sum(u[:])/2, np.sin(r.phi)*np.sum(u[:])/2])
    # Integrate numerically:
    mp = r.mp.copy()
    phi = r.phi
    p = r.p.copy()
    theta = r.theta
    q = r.q.copy()
        
    mp += mp_dot*dt
    phi += phi_dot*dt
    #p += p_dot*dt
    #theta += theta_dot*dt
    #q += r.dq[:]*dt
    q += dq[:]*dt
    p[0] = mp[0] + np.cos(phi)*(r.l[0]*np.cos(q[0]) + r.l[1]*np.cos(q[0]+q[1])) - np.sin(phi)*(r.l[0]*np.sin(q[0]) + r.l[1]*np.sin(q[0]+q[1]))
    p[1] = mp[1] + np.sin(phi)*(r.l[0]*np.cos(q[0]) + r.l[1]*np.cos(q[0]+q[1])) + np.cos(phi)*(r.l[0]*np.sin(q[0]) + r.l[1]*np.sin(q[0]+q[1]))
    theta = phi + q[0] + q[1]
    # Save error for the derivative controller
    prev_error = error
    

    
    # Find position of the first joint (to plot)
    
    # Position of the first joint w.r.t the mobile base:
    p_em = np.array([r.l[0]*np.cos(q[0]),
                     r.l[0]*np.sin(q[0])])
    # a and b are the components associated with dR/dt * p_em (derivative of the rot matrix)
    #a = (-np.sin(phi)*p_em[0] - np.cos(phi)*p_em[1])/(2*r.h)
    #b = (np.cos(phi)*p_em[0] - np.sin(phi)*p_em[1])/(2*r.h)
    
    #J_1 = np.array([[np.cos(phi)/2 - a,np.cos(phi)/2 + a,-np.cos(phi)*r.l[0]*np.sin(q[0]) - np.sin(phi)*r.l[0]*np.cos(q[0])],
    #               [np.sin(phi)/2 - b,np.sin(phi)/2 + b,-np.sin(phi)*r.l[0]*np.sin(q[0]) + np.cos(phi)*r.l[0]*np.cos(q[0])],
    #               [-1/(2*r.h),1/(2*r.h),1]])
    #dp_joint_1 = np.dot(J_1,Q_dot_des[0:3])[0:2]
    
    #p_joint_1 = r.p_joint_1 + dp_joint_1*dt
    
    p_joint_1 = np.zeros(2)
    p_joint_1[0] = mp[0] + np.cos(phi)*p_em[0] - np.sin(phi)*p_em[1]
    p_joint_1[1] = mp[1] + np.sin(phi)*p_em[0] + np.cos(phi)*p_em[1]
    

    
# =============================================================================
#     R = r.rotationMatrix(phi)
#     p_joint_1 = mp + np.dot(R,np.array([(r.l[0]*np.cos(q[0])), r.l[0]*np.sin(q[0])]))
# =============================================================================
    
    # position p based on forward kinematics:
# =============================================================================
#     p =  mp + np.dot(R,np.array([r.l[0]*np.cos(r.q[0]) + r.l[1]*np.cos(r.q[0]+r.q[1]),
#                       r.l[0]*np.sin(r.q[0]) + r.l[1]*np.sin(r.
#                       q[0]+r.q[1])]))
#     
# =============================================================================
    r.Update(mp,phi,p,theta,q,dq,u,p_joint_1) 

    
    storeP = np.append(storeP,[p],axis=0)
# VISUALISE THE MOVEMENT
    if i%10==0:
        plt.cla()
        plt.xlim([-5,5])
        plt.ylim([-5,5])
        #plt.axis('equal')
        ax = plt.gca()
        # Plot the body of the robot:
        robotBody = plt.Circle((mp[0], mp[1]), r.R, color='r',fill=False)
        plt.plot([mp[0],mp[0]+0.8*np.cos(phi)],[mp[1],mp[1]+0.8*np.sin(phi)])
        ax.add_patch(robotBody)
        # Plot link 1:
        plt.plot([mp[0],p_joint_1[0]],[mp[1],p_joint_1[1]],color='orange')
        plt.plot(p_joint_1[0],p_joint_1[1],'.',color='k')
        #
        # Plot link 2:
        #
    # =============================================================================
    #     p_joint_2 = np.array([0,0])
    #     p_joint_2[0] = p_joint_1[0] + r.l[1]*np.cos(r.q[0]+r.q[1])
    #     p_joint_2[1] = p_joint_1[1] + r.l[1]*np.sin(r.q[0]+r.q[1])
    #     plt.plot([p_joint_1[0],p_joint_2[0]],[p_joint_1[1],p_joint_2[1]],color='green')
    # =============================================================================
        plt.plot([p_joint_1[0],p[0]],[p_joint_1[1],p[1]],color='orange')
        # Add the gripper
        angle = np.rad2deg(theta)
        gripper = Arc((p[0]+0.25*np.cos(theta), p[1]+0.25*np.sin(theta)),0.5,0.5,angle+90,0,180, color='r')
        ax.add_patch(gripper)
        #
        
# =============================================================================
#         base_box,polygon1,polygon2 = collisionBox(r, np.array([mp[0],mp[1],phi,q[0],q[1]]))
#         base_x,base_y = base_box.exterior.xy
#         plt.plot(base_x,base_y,'g')
#         x1,y1 = polygon1.exterior.xy
#         x2,y2 = polygon2.exterior.xy
#         
#         poly1 = Polygon([np.array([1,1]),np.array([1,2]),np.array([2,2]),np.array([2,1])])
#         poly2 = Polygon([np.array([3,3]),np.array([4,3]),np.array([4,4]),np.array([3,4])])
#         poly3 = Polygon([np.array([7,7]),np.array([8,7]),np.array([8,8]),np.array([7,8])])
# 
#         
#         plt.plot(poly1.exterior.xy[0],poly1.exterior.xy[1])
#         plt.plot(poly2.exterior.xy[0],poly2.exterior.xy[1])
#         plt.plot(poly3.exterior.xy[0],poly3.exterior.xy[1])
#                 
#         plt.plot(x1,y1)
#         plt.plot(x2,y2)
# =============================================================================
        plt.grid()
        plt.pause(0.1)
        
        if np.all(abs(X-X_des) < 0.01):
            break
    
#plt.show()
plt.plot(storeP[:,0],storeP[:,1],'--')
