from graphics import *
from random import randint
import math
import numpy as np


def draw_room(win):
    environment = Rectangle(Point(0, 0), Point(500, 500))
    environment.setFill('black')
    environment.draw(win)
    border = Rectangle(Point(20, 20), Point(480, 480))
    border.setFill('white')
    border.draw(win)
    start1 = Line(Point(35, 35), Point(45, 45))
    start1.setFill('green')
    start1.setWidth(3)
    start1.draw(win)
    start2 = Line(Point(35, 45), Point(45, 35))
    start2.setFill('green')
    start2.setWidth(3)
    start2.draw(win)
    obstacles = []
    obstacles.append(np.array([[35, 35], [35, 45], [45, 45], [45, 35]]))
    return obstacles


def draw_table(win, obstacles):
    table = Rectangle(Point(380, 420), Point(460, 460))
    table.setFill('white')
    table.setOutline('red')
    table.draw(win)
    objects = []
    for i in range(5):
        y = randint(426, 454)
        ob = Circle(Point(390+i*15, y), 5)
        ob.draw(win)
        objects.append(np.array([390+i*5, y]))
    obstacles.append(np.array([[380, 420], [380, 460], [460, 460], [460, 420]]))
    return objects, obstacles


def draw_obstacles(win, n, obstacles):
    for i in range(n):
        a = randint(40, 460)
        b = randint(40, 460)
        polygonx = []
        polygony = []
        x1 = randint(a-15, a)
        y1 = randint(b-15, b)
        x2 = randint(a-15, a)
        y2 = randint(b, b+15)
        x3 = randint(a, a+15)
        y3 = randint(b, b+15)
        x4 = randint(a, a+15)
        y4 = randint(b-15, b)
        
        l = len(obstacles)
        poly1 = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
        t=0
        for j in range(l):
            poly2 = obstacles[j]
            b=checkPolysIntersecting(poly1, poly2)
            if b:
                break
            else:
                t+=1
                
        if t==l:
                Obs = Polygon(Point(x1, y1), Point(x2, y2),Point(x3, y3), Point(x4, y4))
                Obs.setFill('white')
                Obs.setOutline('blue')
                Obs.draw(win)
                obstacles.append(poly1)
    return obstacles


def checkPolysIntersecting(polyA, polyB):
    """
    Checks whether convex polygons are intersecting by checking 
    if no separation axis from A to B and B to A. 

    Parameters
    ----------
    polyA : np.array(N,2)
        Ordered list of 2D vertices defining a convex polygon
    polyB : np.array(N,2)
        Ordered list of 2D vertices defining a convex polygon

    Returns
    -------
    bool
        True - polygons are intersecting
        False - polgons are not intersecting

    """
    return checkNoSeparation(polyA, polyB) and checkNoSeparation(polyB, polyA)


def checkNoSeparation(polyA, polyB):
    """
    Checks whether separation axis is detected from polyA to polyB 
    (ie checking vertices of A & B against edges of A)

    Parameters
    ----------
    polyA : np.array(N,2)
        Ordered list of 2D vertices defining a convex polygon
    polyB : np.array(N,2)
        Ordered list of 2D vertices defining a convex polygon

    Returns
    -------
    bool
        True - no separation from A to B
        False - separation from A to B

    """

    # Calculate orthogonal axis for each edge in A
    for i in range(len(polyA)):
        vStartIdx = i
        # print("vStartIdx: ", vStartIdx)
        vEndIdx = (i + 1) % len(polyA)
        # print("vEndIdx: ", vEndIdx)
        edge = polyA[vEndIdx, :] - polyA[vStartIdx, :]
        # print("edge: ", edge)
        orthAxis = np.array([-edge[1], edge[0]])
        # print("orthAxis: ", orthAxis)
        orthAxisNorm = np.linalg.norm(orthAxis)
        # print("orthAxisNorm: ", orthAxisNorm)

        # Check minimum projection for points in A
        minProjA = float('inf')
        maxProjA = float('-inf')
        for v in polyA:
            # print("v: ", v)
            proj = np.dot(v, orthAxis)/orthAxisNorm
            # print("proj: ", proj)
            if proj < minProjA:
                minProjA = proj
                # print("minProjA: ", minProjA)
            if proj > maxProjA:
                maxProjA = proj
                # print("maxProjA: ", maxProjA)

        # Check minimum projection for points in B
        minProjB = float('inf')
        maxProjB = float('-inf')
        for v in polyB:
            # print("v: ", v)
            proj = np.dot(v, orthAxis)/orthAxisNorm
            # print("proj: ", proj)
            if proj < minProjB:
                minProjB = proj
                # print("minProjB: ", minProjB)
            if proj > maxProjB:
                maxProjB = proj
                # print("maxProjB: ", maxProjB)

        # Condition for overlap
        if (maxProjA > minProjB) and (minProjA < maxProjB):
            pass
        else:
            # If no overlap detected, exit early

            # DEBUG
            #print("StartIdx, EndIdx, ", vStartIdx, vEndIdx)
            #print("Edge, OrthAxis, ", edge, orthAxis)
            #print("minProjA: ", minProjA)
            #print("maxProjA: ", maxProjA)
            #print("minProjB: ", minProjB)
            #print("maxProjB: ", maxProjA)
            # DEBUG

            return False

    # If all edges checked without separation detected,
    return True


env = GraphWin('Simulation Environment', 500, 500)
env.setCoords(0, 0, 500, 500)
env.setBackground('white')
obstacles = draw_room(env)
[objects, obstacles] = draw_table(env, obstacles)
n_obstacles = randint(120,150)
obstacles = draw_obstacles(env, n_obstacles, obstacles)
env.getMouse()
env.close()
