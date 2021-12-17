from random import seed, randint, uniform
import numpy as np
import matplotlib.pyplot as plt
import collision_detection as cd
import RRT_algorithm as rrt
from robotModel import Robot

def init_room(width=30, height=20, n_obst=20, rng_seed=None):
    
    # Init
    seed(rng_seed)
    wall_thk = 0.5
    obstacles = []  
    
    # Walls
    obstacles.append(np.array([[0, 0], [0, wall_thk], [width, wall_thk], [width, 0]]))
    obstacles.append(np.array([[0, wall_thk], [0, height], [wall_thk, height], [wall_thk, wall_thk]]))
    obstacles.append(np.array([[wall_thk, height], [width, height], [width, height-wall_thk], [wall_thk, height-wall_thk]]))
    obstacles.append(np.array([[width, height-wall_thk], [width, wall_thk], [width-wall_thk, wall_thk], [width-wall_thk, height-wall_thk]]))
    
    # Extra wall in middle
    #obstacles.append(np.array([[width/2, wall_thk], [width/2, height-wall_thk-4], [width/2+wall_thk, height-wall_thk-4], [width/2+wall_thk, wall_thk]]))
    
    # Start - the starting robot configuration. Random y pos on LHS of room, with 0 joint angles
    startPos = np.array([uniform(wall_thk+2.5, wall_thk+7), uniform(wall_thk+2.5, height-wall_thk-2.5)])
    start = np.hstack((startPos, np.array([0.0,0.0,0.0])))
    # Goal - the goal for the end effector (xp, py, theta)
    goal = np.array([uniform(width-wall_thk-2.5, width-wall_thk-7), uniform(wall_thk+2.5, height-wall_thk-2.5), uniform(0, 2*np.pi)])
    
    # Obstacles
    for i in range(n_obst):
        
        # New random poly 
        a = uniform(2*wall_thk, width-2*wall_thk)
        b = uniform(2*wall_thk, height-2*wall_thk)
        x1 = uniform(a-2, a)
        y1 = uniform(b-2, b)
        x2 = uniform(a-2, a)
        y2 = uniform(b, b+2)
        x3 = uniform(a, a+2)
        y3 = uniform(b, b+2)
        x4 = uniform(a, a+2)
        y4 = uniform(b-2, b)
        poly1 = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]])
        
        # Pre emptively add new poly to list
        obstacles.append(poly1)
        polyClear = True
        
        # Check whether too close to start/end point
        tooClose = cd.checkPolyCircleIntersecting(poly1, start[0], start[1], 2.5)
        tooClose = tooClose or cd.checkPolyCircleIntersecting(poly1, goal[0], goal[1], 2.5)
        if tooClose:
            # If too close, 
            polyClear = False

        # For every other obstacle in current list, check for collisions
        l = len(obstacles)-1     
        for j in range(l):
            poly2 = obstacles[j]
            intersecting = cd.checkPolysIntersecting(poly1, poly2)
            if intersecting:
                # If collision detected, poly should be discarded, and stop checking further
                polyClear = False
                break
        
        # Remove from the list if location not clear
        if polyClear == False:
            obstacles.pop()

    return start, goal, obstacles


def draw_room(win, obstacles):
    for obst in obstacles:
        obs = plt.Polygon(obst, color='black', fill=True)
        win.add_patch(obs)
    return


# Initialise room with obstacles and start/goal
room_width = 30
room_height = 20
start, goal, obstacles = init_room(room_width, room_height, n_obst=20, rng_seed=0)

# Draw room
fig, ax = plt.subplots()
ax.set_aspect('equal','box')
ax.set_xlim([0, room_width])
ax.set_ylim([0, room_height])
draw_room(ax, obstacles)
ax.add_patch(plt.Circle(start[0:2],2.5,color='red',fill=False))
ax.add_patch(plt.Circle(goal[0:2],2.5,color='green',fill=False))

# Run RRT
goalConfig = np.array([goal[0],goal[1],0.0,0.0,0.0]) # TODO - convert from endpoint goal to config properly
NodeList = rrt.RRT(start, goalConfig, room_width, room_height, ax, 100, obstacles)

# Get path from tree
currentNode = NodeList[-1]
path = []
path.append(NodeList[-1])
x = []
y = []
while currentNode != NodeList[0]:
    path.append(currentNode.parent)
    currentNode = currentNode.parent
path= path[::-1]

for n in range(len(path)-1):
    ax.plot([path[n].q[0],path[n+1].q[0]],[path[n].q[1],path[n+1].q[1]],color='green')

plt.show()

    
    
    
    