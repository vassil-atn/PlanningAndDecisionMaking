import numpy as np

def findDistance(q1,q2):
    distance = np.linalg.norm(q1[0:2] - q2[0:2]) + np.sin(q1[3]) + np.sin(q1[4])
    
    return distance
def path(q1,q2):
    
    return path

def collisionFree(q1,q2=None):
    
    return collision_free


def RRT(start,goal,N):
    V = [start]
    E = []
    
    for i in range(0,N):
        # Pick a random point in C-space
        while True:
            best_distance = 0
            q = random
            if collisionFree(q) == True:
                break
        # Find closest vertix in V 
        for vertix in V:
            distance = findDistance(vertix,q)
            if distance < best_distance:
                distance == best_distance
                q_closest = vertix
            return q_closest,best_distance
        if collisionFree(q_closest,q) == True:
            V.append(q)
            E.append(best_distance)
        if collisionFree(V[-1],goal) == True:
            break