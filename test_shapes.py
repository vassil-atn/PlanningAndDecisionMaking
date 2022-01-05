import numpy as np
import collision_detection

A = np.array([[-2,1],[2,2],[1,-3]])
B = np.array([[-1,6],[8,7],[9,2],[4,-2]])
C = np.array([[-4,-3],[6,-1],[6,-5],[4,-7]])
D = np.array([[1,-3],[1,-2],[8,-2],[8,-7]])
E = np.array([[-9,2],[-3,7],[-3,2],[-6,-4],[-9,-2]])

shapes = {
    "A": A,
    "B": B,
    "C": C,
    "D": D,
    "E": E
}


for shapeA in shapes:
    for shapeB in shapes:
        print(shapeA, " & ", shapeB)
        print(checkPolysIntersecting(shapes[shapeA],shapes[shapeB]))
