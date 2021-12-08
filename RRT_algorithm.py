import numpy as np
from robotModel import Robot

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
    mp,p_centre1,p_centre2 = r.ForwardKinematicsArm(q)
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