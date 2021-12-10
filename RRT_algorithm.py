import numpy as np
from robotModel import Robot
from shapely.geometry import Polygon,Point
from collision_detection import checkPolysIntersecting

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

def path(q1,q2):
    return path

def collisionFree(r,q,obstacles):

    # Build the bounding boxes for the base and the links:
    poly_base,poly_link1,poly_link2 = collisionBox(r,q)
    
    # Check if in collision:
    collision_free = True
    for obst in obstacles:
        if checkPolysIntersecting(poly_base,obst): # r.R is the radius of the mobile base:
            collision_free = False
            break
        if checkPolysIntersecting(poly_link1,obst): # link1_R is the radius of the circle around first link:
            collision_free = False
            break
        if checkPolysIntersecting(poly_link2,obst): 
            collision_free = False
            break
        
    return collision_free

def collisionPath():
    return 
#
#
# Helper function to define the vertices of the polygons around the robot arm
def buildPolygons(r,link_centre,angle): 
    poly = []
    link_length = r.l[0]
    link_width = 0.2
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
    mp,p_centre1,p_centre2 = r.ForwardKinematicsConfig(q)
    # for the base it's a circle:
    base_box = Point(mp).buffer(0.6) # Create a circle centered at the centre of the mobile base with radius 0.5
    polygon1 = Polygon(buildPolygons(r,p_centre1,r.q[0]+r.phi))
    polygon2 = Polygon(buildPolygons(r,p_centre2,r.q[0]+r.q[1]+r.phi))
    
    return base_box,polygon1,polygon2
#
#   
#
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