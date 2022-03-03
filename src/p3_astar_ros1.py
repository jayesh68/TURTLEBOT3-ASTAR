#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aor 12 08:20:17 2021
@author: jayesh
@teammate: Yoseph Kebede
"""

import numpy as np
import copy
import math
import time
import ast
import cv2
import pygame
import os
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from queue import PriorityQueue
#from act_proj3 import actionSet
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

#Start and Goal Nodes
s = []                      #List storing user input start node
act=[]
g = []                      #List storing user input goal node
n_list=[]
s_list=[]
#Obstacle variables
oblist1=[]                  #List to store the obstacle coordinates for final andimation
riglist=set([])             #List to store obstacle with clearance

#Enironment variables
xmax=1000                   #Width of the map
ymax=1000                    #Height of the map

#Child expansion variables
threshold=0.5                                   #Minimum difference between expanded nodes 
visited=[]                                      #List storing visited nodes
visited_nodes = np.zeros((2002,2002,25))          #Matrix storing visited nodes approximated with respect to threshold
#act=actionSet()                                 #Instance of the actionSet class used to perform the child expansion actions

#Cost storing variables
cost2come = np.full((2002,2002,25),np.inf)        #Matrix storing cost from start to expanded nodes initialized to infinity
cost2goal = np.full((2002,2002,25),np.inf)        #Matrix storing cost from expanded nodes to goal initialized to infinity
totCost = np.full((2002,2002,25),np.inf)          #Matrix storing sum of cost to come and cost to goal initialized to infinity

#Backtracking variables
path_track={}               #Dictionary storing child nodes to a parent key
path_track1={}               #Dictionary storing child nodes to a parent key
#Visualization variables
im_count=0
act_track={}
pygame.init()               #Initializing Pygame
display_width = 1000         #Frame width
display_height = 1000        #Frame height
gameDisplay = pygame.display.set_mode((display_width,display_height),pygame.SCALED)
pygame.display.set_caption('A* Animation')
black = (0,0,0)         #Color represnting the background of image
white = (0,255,255)     #Color respresenting the visited nodes
yellow=(255,255,0)      #Color representing the obstacles

#Temporary Queue variables
q = PriorityQueue()         #Setting a priority queue

def Action(curr_node,ul,ur):
    a_list=[]
    b_list=[]
    x = curr_node[0]
    y = curr_node[1]
    ang=curr_node[2]
    #print('before act',x,y,ang)
    t = 0
    r = 3.8
    l = 35.4
    #print('rpms',ul,ur)
    dt=0.1
    cost=0
    xs=x
    ys=y
    
    while t < 1:
        t = t + dt
        if obstaclecheck(x, y)!=True and x<=xmax and y<=ymax and x>=0 and y>=0:
            xs=x
            ys=y
        
            dx = 0.5*r * (ul + ur) * math.cos(ang*math.pi/180) * dt
            dy = 0.5*r * (ul + ur) * math.sin(ang*math.pi/180) * dt
            dtheta = (r / l) * (ur - ul) * dt
            dtheta=dtheta*180/math.pi
            ang+= dtheta
            cost=cost+ math.sqrt(math.pow((0.5*r * (ul + ur) * math.cos(ang*math.pi/180) * dt),2)+math.pow((0.5*r * (ul + ur) * math.sin(ang*math.pi/180) * dt),2))
            x+= dx
            y+= dy
            
            if obstaclecheck(x, y)!=True and obstaclecheck(xs, ys)!=True and x<=xmax and y<=ymax and x>=0 and y>=0 and xs>0 and ys > 0 and xs<xmax and ys<ymax: 
                plt.plot([xs, x], [ys, y], color="red")
                n_list.append([x,y])
                s_list.append([xs,ys])
                a_list.append([xs,ys])
                b_list.append([x,y])
                pygame.event.get()     
                pygame.display.flip()
                pygame.draw.rect(gameDisplay, white, [xs,1000-ys,1,1])
                pygame.draw.rect(gameDisplay, white, [x,1000-y,1,1])
        else:
            break

        
    if ang >= 360 or ang<0:
        ang=ang%360
        
    new_node = [x,y,int(ang)]        

    return new_node,cost,a_list,b_list


#Function run initially to set the obstacle coordinates in the image and append to a list
def getobstaclespace():
    oblist1=[]       #List to store the obstacle coordinates for final animation
    riglist=set([])
    radius=10
    clearance=5
    dist= radius + clearance
    
    for x in range(0,1001):
        for y in range(0,1001):
            
            if (x-200)**2 + (y-200)**2 <= 100**2:
                oblist1.append([x,y])   
                
            if (x-200)**2 + (y-800)**2 <= 100**2:
                oblist1.append([x,y])
 
            # left square
            if 25 <= x <= 175:
                if 425 <= y <= 575:
                    oblist1.append((x, y))

            # right square
            if 375 <= x <= 625:
                if 425 <= y <= 575:
                    oblist1.append((x, y))

            # top left square
            if 725 <= x <= 875:
                if 200 <= y <= 400:
                    oblist1.append((x, y))
            
            if (x-200)**2 + (y-200)**2 <= (100+dist)**2:
                riglist.add(str([x,y])) 
                
            if (x-200)**2 + (y-800)**2 <= (100+dist)**2:
                riglist.add(str([x,y]))


            # left square
            if (25-dist) <= x <= (175+dist):
                if (425-dist) <= y <= (575+dist):
                    riglist.add(str([x,y]))

            # right square
            if (375-dist) <= x <= (625+dist):
                if (425-dist) <= y <= (575+dist):
                    riglist.add(str([x,y]))

            # top left square
            if (725-dist) <= x <= (875+dist):
                if (200-dist) <= y <= (400+dist):
                    riglist.add(str([x,y]))
    
    return oblist1,riglist
    

def obstaclecheck(x,y):
    radius=10
    clearance=5
    dist= radius + clearance
    
    if (x-200)**2 + (y-200)**2 <= (100+dist)**2:
        return True
                
    if (x-200)**2 + (y-800)**2 <= (100+dist)**2:
        return True

    if (75-dist) <= x <= (175+dist):
        if (425-dist) <= y <= (575+dist):
            return True

    if (375-dist) <= x <= (625+dist):
        if (425-dist) <= y <= (575+dist):
            return True

    if (725-dist) <= x <= (875+dist):
        if (200-dist) <= y <= (400+dist):
            return True
        
    if y>=(ymax-dist) and y<=(ymax):
        return True
    
    if x>=(xmax-dist) and x<=(xmax):
        return True
    
    if x>=0 and x<=dist:
        return True

    if y>=0 and y<=dist:
        return True
    
#Nodes cost calculation
def c2gCalc(start,goal):
    #euclidean distance of goal
    dist=math.sqrt((start[0]-goal[0]) ** 2 + (start[1]-goal[1])**2)
    return dist

#Confirming expanded node has reached goal space
def goalReachCheck(start,goal):
    print('checking goal')
    goal_thresh= 100
    if ((start[0]-goal[0]) ** 2 + (start[1]-goal[1])**2) <= (goal_thresh**2):
        return True
    else:
        return False
    
def round_15(child_ang):
        new_ang = int((round(child_ang/15)*15)//15)
        if new_ang==24:
            new_ang=0
        return new_ang
    
#Creating / Updating total cost of expanded nodes  
def cost_update(child,par,cost,stepprev,stepaft):
    child_ang=int((round(child[2]/15)*15)//15)
    par_ang=int((round(par[1][2]/15)*15)//15)
    x= child[0]; y = child[1]; z=int(child_ang)
    a = par[1][0]; b = par[1][1]; c = int(par_ang)
    x1=int(round(x/2)*2)
    y1=int(round(y/2)*2)
    a1=int(round(a/2)*2)
    b1=int(round(b/2)*2)

    i=0
    while i<len(stepprev):
        if str(stepprev[i]) in path_track1:
            path_track1[str(stepprev[i])].append(stepaft[i])
        else:
            path_track1[str(stepprev[i])]=[]
            path_track1[str(stepprev[i])].append(stepaft[i])
        i+=1
    if ((obstaclecheck(x,y)!=True) and (x>0 and x<xmax) and (y>0 and y<ymax) and (child is not None)):
        if visited_nodes[2*x1][2*y1][z]==1:   
            cost2come[2*x1][2*y1][z]= cost + cost2come[2*a1][2*b1][c]
            totCost1 = cost2come[2*x1][2*y1][z] + cost2goal[2*x1][2*y1][z]
            
            if totCost1 < totCost[2*x1][2*y1][z]:
               totCost[2*x1][2*y1][z] = totCost1
               if str([a,b]) in path_track:
                   path_track[str([a,b])].append([x,y])
               else:
                   path_track[str([a,b])]=[]
                   path_track[str([a,b])].append([x,y])
                                             
        else:       
            visited_nodes[2*x1][2*y1][z]=1
            cost2come[2*x1][2*y1][z]=cost+cost2come[2*a1][2*b1][c]     #Calculating the new cost
            cost2goal[2*x1][2*y1][z]=c2gCalc([x1,y1],[g[0],g[1]])
            totCost[2*x1][2*y1][z]=cost2come[2*x1][2*y1][z]+cost2goal[2*x1][2*y1][z]

            q.put([totCost[2*x1][2*y1][z], [x,y,child[2]]])                   #Updating the priority queue
            child=[x,y,z]

            if str([a,b]) in path_track:
                path_track[str([a,b])].append([x,y])
            else:
                path_track[str([a,b])]=[]
                path_track[str([a,b])].append([x,y])
        
def act_update(step1,step2,rpm1,rpm2):
    i=0
    while i<len(step1):
        act_track[str(step2[i])]=[]
        act_track[str(step2[i])].append([rpm1,rpm2])
        i+=1
        
def main(rpm1,rpm2):
    l=0
    while not q.empty(): #and l!=1:                                        #Process when queue is not empty
        
        a=q.get()                                               #Varibale to store the cost and node position

        x_n= a[1][0]; y_n = a[1][1]; z_n=(round(a[1][2]/15)*15)//15
        
        x_g= g[0]; y_g = g[1]; z_g = int(round(g[2]/15)*15)//15               #g[2]=30 for this project
        
        #Checking if goal is reached or not
        if goalReachCheck([x_n,y_n],[x_g, y_g]):
            print('goal',x_n,y_n,g)
            i=[x_g,y_g]
            visited.append([x_g,y_g])
            path_track[str([x_n,y_n])]=[]
            path_track[str([x_n,y_n])].append(i)
            path_track1[str([x_n,y_n])]=[]
            path_track1[str([x_n,y_n])].append(i)
            print('goal reached')
            break
        
        l+=1
        print(l)
        child1,cost1,step_list1,step_list2 = Action(a[1],0,rpm1)
        cost_update(child1, a, cost1,step_list1,step_list2)
        act_update(step_list1,step_list2,0,rpm1)
        child2,cost2,step_list1,step_list2 = Action(a[1],rpm1,0)
        cost_update(child2, a, cost2,step_list1,step_list2)
        act_update(step_list1,step_list2,rpm1,0)
        child3,cost3,step_list1,step_list2= Action(a[1],rpm1,rpm1)
        cost_update(child3, a, cost3,step_list1,step_list2)
        act_update(step_list1,step_list2,0,rpm1)
        child4,cost4,step_list1,step_list2= Action(a[1],0,rpm2)
        cost_update(child4, a, cost4,step_list1,step_list2)
        act_update(step_list1,step_list2,0,rpm2)
        child5,cost5,step_list1,step_list2 = Action(a[1],rpm2,0)
        cost_update(child5, a, cost5,step_list1,step_list2)
        act_update(step_list1,step_list2,rpm2,0)
        child6,cost6,step_list1,step_list2= Action(a[1],rpm2,rpm2)
        cost_update(child6, a, cost6,step_list1,step_list2)
        act_update(step_list1,step_list2,rpm2,rpm2)
        child7,cost7,step_list1,step_list2 = Action(a[1],rpm1,rpm2)
        cost_update(child7, a, cost7,step_list1,step_list2)
        act_update(step_list1,step_list2,rpm1,rpm2)
        child8,cost8,step_list1,step_list2 = Action(a[1],rpm2,rpm1)
        cost_update(child8, a, cost8,step_list1,step_list2)
        act_update(step_list1,step_list2,rpm2,rpm1)


def plot_ob(path):
        fig, ax = plt.subplots()
        ax.set(xlim=(0, 1000), ylim=(0, 1000))
        c1 = plt.Circle((200, 200), 100, edgecolor = 'k', facecolor = "orange")
        c2 = plt.Circle((200, 800), 100, edgecolor = 'k', facecolor = "orange")
        currentAxis = plt.gca()
        currentAxis.add_patch(Rectangle((25, 425), 150, 150, edgecolor = 'k', facecolor = "orange"))
        currentAxis.add_patch(Rectangle((375, 425), 250, 150, edgecolor = 'k', facecolor = "orange"))
        currentAxis.add_patch(Rectangle((725, 200), 150, 200, edgecolor = 'k', facecolor = "orange"))
            
        ax.add_artist(c1)
        ax.add_artist(c2)
        ax.set_aspect('equal')
        plt.grid()
        
        plt.plot(g[0], g[1], color='green', marker='o', linestyle='dashed', linewidth=30,
        markersize=30)
        plt.plot(s[0], s[1], color='yellow', marker='o', linestyle='dashed', linewidth=30,
        markersize=30)
        path = path[::-1]
        x_path = [path[i][0] for i in range(len(path))]
        y_path = [path[i][1] for i in range(len(path))]
        plt.plot(x_path, y_path, "-r",linewidth=3.4)
        l=0
        while l<len(s_list):
            plt.plot([s_list[l][0], n_list[l][0]], [s_list[l][1], n_list[l][1]],linewidth=1, color="blue")
            l=l+1
           
def backtracking (start, goal):         #Backtracking to find the paths traversed from the initial state to the final state
    val = goal
    path_track_list=[]

    path_track_list.append(val)
    try:
        if str('[152, 1]') in path_track1.keys():
            print('found')
        else:
            print('key no')
        if '[208, 2]' in path_track1.values():
            print('val found')
        else:
            print('not found')
        #print(path_track1.keys())
        while val!=start:
            for key, values in path_track1.items():

                while val in values:
                    key= ast.literal_eval(key)                  #converting strings of lists to pure lists

                    val = key
                    path_track_list.append(val)

        
        path_track_list1 = path_track_list[::-1]
        for path in path_track_list1:
            for key in act_track.keys():
                if str(path)==key:
                    act.append(act_track.get(key))
        print('Action set',act_track)
        print('final act list',act)   

    except KeyError:
        print('value not found')
    plot_ob(path_track_list)  
    
    return path_track_list

def ros_inputs(Thetai, UL, UR):
    r = 0.038 #in metres
    L = 0.354 #in metres

    UL = UL*2*math.pi/60
    UR = UR*2*math.pi/60

    
    thetan = 3.14 * Thetai / 180

    theta_dot = (r / L) * (UR - UL) 
    change_theta = theta_dot + thetan

    x_dot = (r / 2) * (UL + UR) * math.cos(change_theta) 
    y_dot = (r / 2) * (UL + UR) * math.sin(change_theta) 

    vel_mag = math.sqrt(x_dot** 2 + y_dot** 2)  

    return vel_mag, theta_dot

def ros_pub(path, act):
    rospy.init_node('astar_search', anonymous=True)
    velPub = rospy.Publisher('cmd_vel', Twist, queue_size=100)
    msg = Twist()

    turtlebot3_model = rospy.get_param("model", "waffle")

    msg.linear.x = 0.0
    msg.linear.y = 0.0
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = 0.0

    velPub.publish(msg)

    c =0
    r = rospy.Rate(10)

    for a in act:
        a=a[0]
        while not rospy.is_shutdown():
            if c== 11:
                msg.linear.x = 0
                msg.angular.z = 0
                velPub.publish(msg)
                break
            else:
                print('publishing')
                print('act',a[0],a[1])
                vel , th = ros_inputs(30,a[0],a[1])
                msg.linear.x = vel*1.5
                msg.angular.z =  th
                velPub.publish(msg)
                c=c+1
                r.sleep()
        c=0
        
def visualization():                            #Creating an animation using pygame 
    pygame.event.get()    
    gameDisplay.fill(black)

    #Setting the obstacle space in the animation
    for path in oblist1:
        x = int(path[0])
        y = abs(1000-int(path[1]))
        #pygame.display.flip()
        pygame.draw.rect(gameDisplay, yellow, [x,y,1,1])
        #pygame.time.wait(0)
    
##################################################### Code execution starts here #######################################################
if __name__ == "__main__":

    oblist1, riglist=getobstaclespace()                         #Retrieve obstacle and clearance information
    visualization()
    #act.plot_ob()
    while True:
        # x1=20               #int(input('Enter x coordinate of start node: '))
        # y1=20               #int(input('Enter y coordinate of start node: '))
        # theta=30      #int(input('Enter degree of start node: '))           #theta for this project is 30
        x1=600
        y1=800
        theta=30
        s = [x1,y1,theta]                                              #Start Position
        rpm1=int(input('Enter rpm of left wheel: '))
        rpm2=int(input('Enter rpm of right wheel: '))
        x2=int(input('Enter x coordinate of goal node: '))
        y2=int(input('Enter y coordinate of goal node: '))
        g = [x2,y2,30]                                              #Goal Position

        start= [x1, y1]
        goal = [x2, y2]
        act_track[str(start)]=[]
        act_track[str(start)].append([0,0])
        
        if goalReachCheck(start,goal):                              #Checking if goal node is the same as the start node
            print('start node equal to/within threshold of goal node. Re enter your points again')
            continue

        elif obstaclecheck(x1,y1)==True:                                  #Checking if start node is in the obstaclespace plus clearance
            print('Start node in obstacle space. Re enter the points again')
            continue
        
        elif obstaclecheck(x2,y2)==True:                                  #Checking if goal node is in the obstaclespace plus clearance
            print('Goal node in obstacle space. Re enter the points again')
            continue

        elif (x1 <0 or x1> xmax) or (y1<0 or y1 > ymax):    #Checking if start node is within the grid(400x300)
            print('start node is outside environment. Re enter the points again')
            continue 

        elif (x2 <0 or x2> xmax) or (y2<0 or y2 > ymax):        #Checking if goal node is within the grid(400X300)
            print('Goal node is outside Environment. Re enter the points again')
            continue

        else:
            break
        
    print(s)
    print(g)
    
    visited.append(start)
    z=int(round(s[2]/15)*15)//15

    visited_nodes[2*x1][2*y1][z]=1

    #Initializing the cost to come of start point to zero
    cost2come[2*x1][2*y1][z] = 0
    
    #Initializing the cost2goal for start node
    cost2goal[2*x1][2*y1][z] =  c2gCalc(start,goal)
    
    #Updating the total heurisitc for start node
    totCost[2*x1][2*y1][z] = cost2come[2*x1][2*y1][z] + cost2goal[2*x1][2*y1][z]
    #print(totCost[2*x1][2*y1][z])
    #Initializing the queue with a Total cost and the start node
    q.put([totCost[2*x1][2*y1][z], s])              
    
    start_time = time.time()            #Program start time
    main(rpm1,rpm2)                              #Executing search
    
    #Time to reach goal state
    print('time to reach goal',time.time()-start_time)
    
    #Converting start and goal node with threshold of 0.5 units
    s=[x1,y1]
    g=[x2,y2]
    print(g)

    #print('path track',path_track)
    #Performing backtracking to obtain list for optimal path
    path_track_list = backtracking(s, g)
    
    path1=[]
    #print(path)
    
    #print('path track',path_track_list)
    for path in path_track_list:
        pygame.event.get()
        x = path[0]
        y = abs(1000-path[1])
        path1.append((x,y))
        pygame.display.flip()
        #print('displaying')
        #print(x,y)
        pygame.draw.rect(gameDisplay, (255,5,5), [x,y,5,5])
        #pygame.image.save(gameDisplay, f"/home/jayesh/Documents/ENPM661_PROJECT1/map1/{im_count}.png")         #Saving the images to create a video                                                                                         #uncomment if not required
        #im_count+=1
        pygame.time.wait(20)
    plt.show()
    
    ros_pub(path_track_list,act)

    #Terminate Pygame
    pygame.quit()

    #Print the total time taken to reach goal state and backtrack
    print("total time:")
    print(time.time()-start_time)


    
############################################# Code Execution ends here #############################################################
