import numpy as np

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

