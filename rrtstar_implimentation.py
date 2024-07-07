
import sys, random, math, pygame
from pygame.locals import *
from math import sqrt,cos,sin,atan2
from lineIntersect import *
from scipy.interpolate import * 
import numpy as np

#global constants
XDIM = 640
YDIM = 480
WINSIZE = [XDIM, YDIM]
EPSILON = 8.0
NUMNODES = 5000
RADIUS=15
# OBS=[(500,150,100,50),(300,80,100,50),(150,220,100,50),(100,100,100,100)]
OBS = []
dilated_OBS = []

#function to draw starting point and goal point
def obsDraw(pygame,screen,start,goal):
    blue=(0,0,255)
    radius = 10
    green = (0,255,0)
    red = (255,0,0)
    pygame.draw.circle(screen, green, start, radius)
    pygame.draw.circle(screen, red, goal, radius)
    for o in OBS: 
      pygame.draw.rect(screen,blue,o)
      

def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

def chooseParent(nn,newnode,nodes):
        for p in nodes:
         if checkIntersect(p,newnode,OBS) and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and p.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y]):
          nn = p
        newnode.cost=nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y])
        newnode.parent=nn
        return newnode,nn

def reWire(nodes,newnode,pygame,screen):
        white = 255, 240, 200
        black = 20, 20, 40
        for i in range(len(nodes)):
           p = nodes[i]
           if checkIntersect(p,newnode,OBS) and p!=newnode.parent and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < p.cost:
              pygame.draw.line(screen,white,[p.x,p.y],[p.parent.x,p.parent.y])  
              p.parent = newnode
              p.cost=newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) 
              nodes[i]=p  
              pygame.draw.line(screen,black,[p.x,p.y],[newnode.x,newnode.y])                    
        return nodes

#function to optimize the path and regenerate a shorter path by short cutting
def shortcut_path(path, iterations=100):
    for _ in range(iterations):
        index1, index2 = sorted(random.sample(range(len(path)), 2))
        point1,point2 = path[index1],path[index2]
        if checkIntersectsmooth(point1,point2,dilated_OBS):
            path = path[:index1+1]+[point2]+path[index2+1:]
    return path
        
def dilate_OBS(OBS):
    dilation_amount = 5
    for rect in OBS:
        x, y, w, h = rect
        new_x = x - dilation_amount
        new_y = y - dilation_amount
        new_w = w + 2 * dilation_amount
        new_h = h + 2 * dilation_amount
        dilated_OBS.append((new_x, new_y, new_w, new_h))
    return dilated_OBS   

#function to draw the path once the nodes created are finished
def drawSolutionPath(start, goal, nodes, pygame, screen):
    pink = 200, 20, 240
    orange = 255,165,0
    green = 0,255,0
    ####################################################
    path_points = []

    # Trace back from goal to start to collect path points
    nn = nodes[0]
    for p in nodes:
        if dist([p.x, p.y], [goal.x, goal.y]) < dist([nn.x, nn.y], [goal.x, goal.y]):
            nn = p
    while nn != start:
        path_points.append([nn.x, nn.y])
        nn = nn.parent
    path_points.append([start.x, start.y])
    smooth_path = shortcut_path(path_points)

    # Fit a spline through the collected path points
    path_points = np.array(path_points)
    smooth_path_points = np.array(smooth_path)
    tck, u = splprep(path_points.T, s=0, k=3)
    u_new = np.linspace(u.min(), u.max(), 1000)
    x_spline, y_spline = splev(u_new, tck)


    # Draw the smoothed path using the spline
    for i in range(len(x_spline) - 1):
        pygame.draw.line(screen, orange, (int(x_spline[i]), int(y_spline[i])), (int(x_spline[i + 1]), int(y_spline[i + 1])), 3)

    ####################################################
    nn = smooth_path[0]
    # for p in path_points:
    #     if dist([p[0], p[1]], [goal.x, goal.y]) < dist([nn[0], nn[1]], [goal.x, goal.y]):
    #         nn = p
    # while nn != (start.x,start.y):
    #     pygame.draw.line(screen, pink, [nn[0], nn[1]], [nn.parent.x, nn.parent.y], 5)
    #     nn = nn.parent

    ####draws the shortest distance path, but not the smoothest in the curves
    for i in range(len(smooth_path) - 1):
      pygame.draw.line(screen, pink, smooth_path[i], smooth_path[i + 1], 3)

    

class Cost:
    x = 0
    y = 0
    cost=0  
    parent=None
    def __init__(self,xcoord, ycoord):
         self.x = xcoord
         self.y = ycoord

class Node:
    x = 0
    y = 0
    cost=0  
    parent=None
    def __init__(self,xcoord, ycoord):
         self.x = xcoord
         self.y = ycoord


#define function for creating obstacles once the start and end coordinates are given from the mouse
def create_obstacle(start_pos, end_pos):
    x1, y1 = start_pos
    x2, y2 = end_pos
    return pygame.Rect(min(x1, x2), min(y1, y2), abs(x2 - x1), abs(y2 - y1))

	
def main():

    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('Induwara_RRtar for Vega')
    white = 255, 255, 255
    black = 20, 20, 40
    

    nodes = []
    ############INSERT START NODE HERE###############
    nodes.append(Node(30.0,30.0))
    start=nodes[0]
    ################INSERT GOAL NODE HERE##################
    goal=Node(530.0,370.0)

    screen.fill(white)
    obsDraw(pygame,screen,(start.x,start.y),(goal.x,goal.y))
   
    drawing_obstacle = False
    start_pos = None
    
    while True:
      
      for event in pygame.event.get():
          if event.type == QUIT:
            pygame.quit()
            sys.exit()
          elif event.type == MOUSEBUTTONDOWN:
            drawing_obstacle = True
            start_pos = event.pos
            screen.fill(white)
            obsDraw(pygame,screen,(start.x,start.y),(goal.x,goal.y))
          elif event.type == MOUSEBUTTONUP:
            drawing_obstacle = False
            end_pos = event.pos
            new_obs = create_obstacle(start_pos, end_pos)
            OBS.append(new_obs)
            

          elif event.type == KEYDOWN and event.key == K_RETURN:

            dilated_OBS = dilate_OBS(OBS)
            for i in range(NUMNODES):
                rand = Node(random.random()*XDIM, random.random()*YDIM)
                nn = nodes[0]
                for p in nodes:
                  if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
                    nn = p
                interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
          
                newnode = Node(interpolatedNode[0],interpolatedNode[1])
                if checkIntersect(nn,rand,dilated_OBS):
                  
                  [newnode,nn]=chooseParent(nn,newnode,nodes)
              
                  nodes.append(newnode)
                  pygame.draw.line(screen,black,[nn.x,nn.y],[newnode.x,newnode.y])
                  nodes=reWire(nodes,newnode,pygame,screen)
                  pygame.display.update()
                

                  for e in pygame.event.get():
                    if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                        sys.exit("Leaving because you requested it.")
            drawSolutionPath(start,goal,nodes,pygame,screen)
            

      if drawing_obstacle:
            current_pos = pygame.mouse.get_pos()
            temp_obs = create_obstacle(start_pos, current_pos)
            pygame.draw.rect(screen, (0, 0, 255), temp_obs, 1)

      pygame.display.update()

if __name__ == '__main__':
    main()
    # running = True
    # while running:
    #    for event in pygame.event.get():
    #        if event.type == pygame.QUIT:
    #              running = False



