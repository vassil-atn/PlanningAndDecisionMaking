from random import randint
import math
import numpy as np
import matplotlib.pyplot as plt

def draw_room(env):
    environment=plt.Rectangle([0,0],500,500,color='black')
    env.add_patch(environment)
    border=plt.Rectangle([20,20],460,460,color='white')
    env.add_patch(border)
    start1=plt.Line2D([35,45],[35,45],linewidth=2,color='green')
    env.add_line(start1)
    start2=plt.Line2D([35,45],[45,35],linewidth=2,color='green')
    env.add_line(start2)
    occupance=[]
    occupance=append_points(occupance,0,0,500,20)
    occupance=append_points(occupance,0,0,20,500)
    occupance=append_points(occupance,0,480,500,500)
    occupance=append_points(occupance,480,0,500,500)
    occupance=append_points(occupance,380,420,460,460)
    occupance=append_points(occupance,35,35,45,45)
    wall1=plt.Rectangle([220,0],20,200,color='black')
    env.add_patch(wall1)
    occupance=append_points(occupance,220,0,240,200)
    wall2=plt.Rectangle([350,180],150,20,color='black')
    env.add_patch(wall2)
    occupance=append_points(occupance,350,180,500,200)
    wall3=plt.Rectangle([370,350],130,20,color='black')
    env.add_patch(wall3)
    occupance=append_points(occupance,370,350,500,370)
    wall4=plt.Rectangle([250,300],20,200,color='black')
    env.add_patch(wall4)
    occupance=append_points(occupance,250,300,270,500)
    wall5=plt.Rectangle([0,300],150,20,color='black')
    env.add_patch(wall5)
    occupance=append_points(occupance,0,300,150,320)
    wall6=plt.Rectangle([0,180],150,20,color='black')
    env.add_patch(wall6)
    occupance=append_points(occupance,0,180,150,200)
    return occupance

def append_points(occupance,x1,y1,x2,y2):
    dx=abs(x1-x2)+1
    dy=abs(y1-y2)+1
    for i in range(int(dx)):
        for j in range(int(dy)):
            occupance.append([x1+i,y1+j])
    return occupance


def draw_table(env):
    table=plt.Rectangle([380,420],80,40,color='red',fill=False)
    env.add_patch(table)
    for i in range(5):
        y=randint(427,453)
        objects=plt.Circle((390+i*15,y),5,color='red',fill=False)
        env.add_patch(objects)

def draw_obstacles(env, n, occupied):
    x_list = []
    y_list = []
    r_list = []
    obstacle1 = plt.Circle((250, 250), 20,color='b')
    x_list.append(250)
    y_list.append(250)
    r_list.append(20)
    env.add_patch(obstacle1)
    for i in range(n-1):
        x = randint(20, 480)
        y = randint(20, 480)
        r = randint(10, 15)
        b1 = check_distance(x_list, y_list, r_list, x, y, r)
        b2 = check_overlap(x,y,r,occupied)
        if b1 and b2:
            obstacle=plt.Circle((x,y), r,color='b')
            x_list.append(x)
            y_list.append(y)
            r_list.append(r)
            env.add_patch(obstacle)
            
def check_overlap(x,y,r,occupied):
    n=len(occupied)
    t=0
    for i in range(n):
        p=occupied[i]
        px=p[0]
        py=p[1]
        if distance(x,y,px,py)<=r+4:
            return False
            break
        else:
            t+=1
    if t==n:
         return True
    else:
         return False
            

def check_distance(x_list, y_list, r_list, x, y, r):
    n = len(x_list)
    t = 0
    for i in range(n):
        x1 = x_list[i]
        y1 = y_list[i]
        r1 = r_list[i]
        if distance(x1, y1, x, y) <= r+r1+10:
            return False
            break
        else:
            t += 1
    if t == n:
        return True
    else:
        return False


def distance(x1, y1, x2, y2):
    distance = math.sqrt(((x1-x2)**2)+((y1-y2)**2))
    return distance


plt.figure(1)
plt.xlim([0, 500])
plt.ylim([0, 500])
env = plt.gca()
occupied=draw_room(env)
draw_table(env)
n_obstacles = randint(100, 150)
draw_obstacles(env, n_obstacles, occupied)

