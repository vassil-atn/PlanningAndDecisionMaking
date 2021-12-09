from graphics import *
from random import randint
import math
import numpy as np


class Obstacle:
    def __init__(self,window,centre,radius):
        self.obstacle = Circle(centre,radius)
        self.obstacle.setFill('blue')
        self.obstacle.draw(window)
        self.radius=radius
        self.centre=centre
        self.centreX=centre.getX()
        self.centreY=centre.getY()
        
    def returnCentre(self):
        return self.centre
        
    def returnCentreX(self):
        return self.centreX
    
    def returnCentreY(self):
        return self.centreY

    def returnRadius(self):
        return self.radius
    
def draw_room(win):
    environment=Rectangle(Point(0,0),Point(500,500))
    environment.setFill('black')
    environment.draw(win)
    border=Rectangle(Point(20,20),Point(480,480))
    border.setFill('white')
    border.draw(win)
    start1=Line(Point(35,35),Point(45,45))
    start1.setFill('green')
    start1.setWidth(3)
    start1.draw(win)
    start2=Line(Point(35,45),Point(45,35))
    start2.setFill('green')
    start2.setWidth(3)
    start2.draw(win)
    occupance=[]
    occupance=append_points(occupance,Point(0,0),Point(500,20))
    occupance=append_points(occupance,Point(0,0),Point(20,500))
    occupance=append_points(occupance,Point(0,480),Point(500,500))
    occupance=append_points(occupance,Point(480,0),Point(500,500))
    occupance=append_points(occupance,Point(380,420),Point(460,460))
    occupance=append_points(occupance,Point(35,35),Point(45,45))
    wall1=Rectangle(Point(220,0),Point(240,200))
    wall1.setFill('black')
    wall1.draw(win)
    occupance=append_points(occupance,Point(220,0),Point(240,200))
    wall2=Rectangle(Point(350,180),Point(500,200))
    wall2.setFill('black')
    wall2.draw(win)
    occupance=append_points(occupance,Point(350,180),Point(500,200))
    wall3=Rectangle(Point(370,350),Point(500,370))
    wall3.setFill('black')
    wall3.draw(win)
    occupance=append_points(occupance,Point(370,350),Point(500,370))
    wall4=Rectangle(Point(250,300),Point(270,500))
    wall4.setFill('black')
    wall4.draw(win)
    occupance=append_points(occupance,Point(250,300),Point(270,500))
    wall5=Rectangle(Point(0,300),Point(150,320))
    wall5.setFill('black')
    wall5.draw(win)
    occupance=append_points(occupance,Point(0,300),Point(150,320))
    wall6=Rectangle(Point(0,180),Point(150,200))
    wall6.setFill('black')
    wall6.draw(win)
    occupance=append_points(occupance,Point(0,180),Point(150,200))
    return occupance

def append_points(occupance,point1,point2):
    dx=abs(point1.getX()-point2.getX())+1
    dy=abs(point1.getY()-point2.getY())+1
    for i in range(int(dx)):
        for j in range(int(dy)):
            occupance.append(Point(point1.getX()+i,point1.getY()+j))
    return occupance
    
    
def draw_table(win):
    table=Rectangle(Point(380,420),Point(460,460))
    table.setFill('white')
    table.setOutline('red')
    table.draw(win)
    objects=[]
    for i in range(5):
        y=randint(426,454)
        objects.append(Circle(Point(390+i*15,y),5))
        objects[i].draw(win)
        
    
def draw_obstacles(win,n,occupied):
    obstacles=[]
    obstacles.append(Obstacle(win,Point(250,250),20))
    for i in range(n-1):
        x=randint(20,480)
        y=randint(20,480)
        r=randint(10,15)
        b1=check_distance(obstacles,x,y,r)
        b2=check_overlap(x,y,r,occupied)
        if b1 and b2:
            obstacles.append(Obstacle(win,Point(x,y),r))
    

def check_distance(obstacles,x,y,r):   
    n=len(obstacles) 
    t=0
    for i in range(n):
        obs=obstacles[i]
        x1=obs.returnCentreX()
        y1=obs.returnCentreY()
        r1=obs.returnRadius()
        if distance(x1,y1,x,y)<=r+r1+10:
            return False
            break
        else:
            t+=1
    if t==n:
         return True
    else:
         return False
     
def check_overlap(x,y,r,occupied):
    n=len(occupied)
    t=0
    for i in range(n):
        p=occupied[i]
        px=p.getX()
        py=p.getY()
        if distance(x,y,px,py)<=r+2:
            return False
            break
        else:
            t+=1
    if t==n:
         return True
    else:
         return False
    
    
def distance(x1,y1,x2,y2):
    distance=math.sqrt(((x1-x2)**2)+((y1-y2)**2))
    return distance

         
env= GraphWin('Simulation Environment',500,500)   
env.setCoords(0,0,500,500)     
env.setBackground('white') 
occupied=draw_room(env)
draw_table(env)  
n_obstacles=randint(100,150)
draw_obstacles(env,n_obstacles,occupied)
env.getMouse()
env.close()