import numpy as np

def checkPolyCircleIntersecting(poly, circleX, circleY, circleR):
    """
    Checks whether a polygon intersects a circle
    
    Parameters
    ----------
    poly : np.array(N,2)
        Ordered list of 2D vertices defining a convex polygon
    circleX, circleY : float
        Circle centre coordinates
    circleR : float
        Circle radius

    Returns
    -------
    bool
        True - intersecting
        False - not intersecting

    """
    
    # Check if any line in poly collides with the circle
    for i in range(len(poly)):
        vStartIdx = i
        # print("vStartIdx: ", vStartIdx)
        vEndIdx = (i + 1) % len(poly)
        # print("vEndIdx: ", vEndIdx)
        
        # Return intersecting True on first line collision
        if checkLineCircleIntersecting(poly[vStartIdx,0], poly[vStartIdx,1],
                                       poly[vEndIdx,0], poly[vEndIdx,1], 
                                       circleX, circleY, circleR):
            return True
    
    # No collision if all edges checked
    return False


def checkLineCircleIntersecting(X1, Y1, X2, Y2, circleX, circleY, circleR):
    """
    Checks whether a line intersects a circle
    
    Parameters
    ----------
    X1, Y1, X2, Y2: float
        Start and end of line coordinates
    circleX, circleY : float
        Circle centre coordinates
    circleR : float
        Circle radius

    Returns
    -------
    bool
        True - intersecting
        False - not intersecting

    """
    
    vStart =  np.array([X1, Y1])
    #print("vStart: ", vStart)
    vEnd =  np.array([X2, Y2])   
    #print("vEnd: ", vEnd)
    centre = np.array([circleX, circleY])
    line = vEnd - vStart
    
    # Check whether either end is inside circle
    if np.linalg.norm([vStart[0]-circleX, vStart[1]-circleY]) < circleR:
        return True
    elif np.linalg.norm([vEnd[0]-circleX, vEnd[1]-circleY]) < circleR:
        return True
    
    # Get projection size from edge start to circle centre onto edge
    proj = np.dot(centre - vStart, line)
    #print("proj: ", proj)
    projFraction = proj/np.dot(line, line)
    # if projection fraction is < 0 or > 1, closest point is off line (no collision)
    if projFraction < 0 or projFraction > 1:
        return False
    
    # Closest point on line
    vClose = vStart + projFraction*line
    # Distance closest point to centre
    distClose = np.linalg.norm(centre - vClose)
    
    if distClose < circleR:
        return True
    else:
        return False


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
        edge = polyA[vEndIdx,:] - polyA[vStartIdx,:]
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
            proj = np.dot(v,orthAxis)/orthAxisNorm
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
            proj = np.dot(v,orthAxis)/orthAxisNorm
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
            
            ## DEBUG
            #print("StartIdx, EndIdx, ", vStartIdx, vEndIdx)
            #print("Edge, OrthAxis, ", edge, orthAxis)
            #print("minProjA: ", minProjA)
            #print("maxProjA: ", maxProjA)
            #print("minProjB: ", minProjB)
            #print("maxProjB: ", maxProjA)
            ## DEBUG
            
            return False
        
    # If all edges checked without separation detected,
    return True

