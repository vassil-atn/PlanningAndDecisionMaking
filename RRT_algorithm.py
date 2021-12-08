import numpy as np
from robotModel import Robot
from shapely.geometry import Polygon,Point


def HaarMeasure(angle1,angle2):
    absolute = abs(angle1-angle2)
    if  absolute >= -np.pi and absolute <= np.pi:
        return absolute
    elif absolute < -np.pi:
        return (absolute + 2*np.pi)
    elif absolute > np.pi:
        return (absolute - 2*np.pi)


def findDistance(q1,q2):
    # Use Haar Measure for the angles:
    distance = np.linalg.norm(q1[0:2] - q2[0:2]) + HaarMeasure(q1[2],q2[2]) + HaarMeasure(q1[3],q2[3])
    
    return distance

def path(q1,q2):
    return path

def collisionFree(r,q,obstacles):
    link1_R = r.l[0]/2
    link2_R = r.l[1]/2

    # Convert the configuration to workspace:
    mp,p_centre1,p_centre2 = r.ForwardKinematicsConfig(q)
    # Check if in collision based on circles:
    collision_free = True
    for obst in obstacles:
        if np.linalg.norm(mp - obst.p) < (r.R + obst.R): # r.R is the radius of the mobile base:
            collision_free = False
            break
        if np.linalg.norm(p_centre1 - obst.p) < (link1_R + obst.R): # link1_R is the radius of the circle around first link:
            collision_free = False
            break
        if np.linalg.norm(p_centre2 - obst.p) < (link2_R + obst.R): 
            collision_free = False
            break
        
    return collision_free

def collisionPath():
    return 
#Ordered list of 2D vertices defining a convex polygon
def buildPolygons(r,link_centre,angle): # This function should build polygons around the robot arms
    poly = []
    link_length = r.l[0]
    link_width = 0.2
    R = r.rotationMatrix(angle)
    poly.append(link_centre + R.dot(np.array([-link_length/2,link_width/2])))
    poly.append(link_centre + R.dot(np.array([link_length/2,link_width/2])))
    poly.append(link_centre + R.dot(np.array([link_length/2,-link_width/2])))
    poly.append(link_centre + R.dot(np.array([-link_length/2,-link_width/2])))
    return poly
def collisionBox(r,q): # This function calculates the overall collision box for the robot (base+arms)
    mp,p_centre1,p_centre2 = r.ForwardKinematicsConfig(q)
    # for the base it's a circle:
    base_box = Point(mp).buffer(0.6) # Create a circle centered at the centre of the mobile base with radius 0.5
    polygon1 = Polygon(buildPolygons(r,p_centre1,r.q[0]+r.phi))
    polygon2 = Polygon(buildPolygons(r,p_centre2,r.q[0]+r.q[1]+r.phi))
    
    return base_box,polygon1,polygon2
    

def RRT(start,goal,N):
    V = [start]
    E = []
    r = Robot()
    for i in range(0,N):
        # Pick a random point in C-space
        while True:
            best_distance = 0
            q = random
            if collisionFree(r,q) == True:
                break
        # Find closest vertix in V 
        for vertix in V:
            distance = findDistance(vertix,q)
            if distance < best_distance:
                distance == best_distance
                q_closest = vertix
            return q_closest,best_distance
        if collisionFree(r,q_closest,q) == True:
            V.append(q)
            E.append(best_distance)
        if collisionFree(r,V[-1],goal) == True:
            break