# -*- coding: utf-8 -*-
"""
Created on Sat May 23 21:33:43 2020

@author: user
"""

#generate data to read for navigation

import math
import numpy as np
import cvxpy as cp
import copy

#ROBOT DIMENSIONS
neutralAnkle = math.radians(30)
roboRadius = 0.2*math.sqrt(2) #radius of robot from geometric center to hip joint
l1=0.2*math.sqrt(2) #length of first leg segment after hip joint
l2=0.4*math.sqrt(2) #length of second seg segment (after ankle joint)
neutralRadiusTotal = l2*math.cos(neutralAnkle)+l1+roboRadius #radius of pitch circle

#DECLARATION OF VALUES FOR NAVIGATION

NavigatioN = 120 #total number of steps in navigation
pInitial = [0, 0] #Coordinates of initial position of agent
pFinal = [30,8] #coordinates of goal location

pObs = np.array([[5,2],[12,6],[12,1],[20,3],[26,6],[26,1]]) #positions of center of obstacles
rObs = [2.06,1.78,1.78,3.81,1.66,1.56] #radii of obstacles

obstacles = len(rObs) #total number of obstacles to dodge


#SOLVING TRAJECTORY AS CONVEX PROBLEM
pL = cp.Variable((NavigatioN,2)) #number of steps, number of dimensions, left trajectory
pR = cp.Variable((NavigatioN,2)) #number of steps, number of dimensions, right trajectory

gasolina = cp.expressions.expression.Expression #number 

gasolina = 0
for i in range(1,NavigatioN-1): #intermediate steps
    gasolina += cp.norm(pL[i-1,:]-2*pL[i,:]+pL[i+1,:],2)**2 #calculation of squared 2-norms to be added for left trajectory
    gasolina += cp.norm(pR[i-1,:]-2*pR[i,:]+pR[i+1,:],2)**2 #squared 2-norms for right trajectory
         
objectNav = cp.Minimize(gasolina)

constrNav = [] #initialize constraints list
 
constrNav += [pL[0,:] == pInitial] #left trajectory
constrNav += [pL[NavigatioN-1,:] == pFinal]
constrNav += [pR[0,:] == pInitial] #right trajectory
constrNav += [pR[NavigatioN-1,:] == pFinal]

    
for o in range(0,obstacles):
    if pInitial[0] <= pObs[o,0] - rObs[o] and pInitial[1] <= pObs[o,1]+rObs[o]: #if there are obstacles in the way
        #first trajectory
        constrNav += [pL[NavigatioN/2,0] <= pObs[o,0]-rObs[o]] #passing on the left
        constrNav += [pL[NavigatioN/2,1] >= pObs[o,1]+rObs[o]] #passing above
        #second trajectory
        constrNav += [pR[NavigatioN/2,0] >=pObs[o,0] + rObs[o]] #passing on the right
        constrNav += [pR[NavigatioN/2,1] <= pObs[o,1] -rObs[o]] #passing below

problem = cp.Problem(objectNav,constrNav)
optval = problem.solve(solver=cp.ECOS)
#END OF CONVEX OPTIMIZATION PART
#print(objectNav)
print(optval)
#print(pL.value)
#print(pR.value)

dieseL = 0
dieseR = 0
for g in range(1,NavigatioN-1): #selecting which path to take
    dieseL += np.linalg.norm(pL.value[g-1,:]-2*pL.value[g,:]+pL.value[g+1,:],2)**2
    dieseR += np.linalg.norm(pR.value[g-1,:]-2*pR.value[g,:]+pR.value[g+1,:],2)**2
    
if dieseL <= dieseR:
    path = copy.deepcopy(pL.value)
    print('left')
else:
    path = copy.deepcopy(pR.value)
    print('right')
#print(path)
    
#def nextTarget(i,path,botPosition,navRadius): #arguments are index in path vector, path vector, current location of bot (3d), radius of bot for navigation
#    print('current location: ',botPosition[:2])
#    print('vector to target: ',path[i]-botPosition[:2])
#    print('distance to target: ',np.linalg.norm(path[i]-botPosition[:2]))
#    if i >= len(path)-1: #we are using the goal location
#        return len(path)-1 #use the goal location  
#    path_subset = path[i:] #vector to hold future elements of the path
#    #twoDist = math.inf #start at unreasonable value, to overwrite
#    for step in range(len(path_subset)-1,-1,-1): #iterate backwards over next target positions
#        twoDist = np.linalg.norm(path[step]-botPosition[:2])
#        if twoDist <= navRadius: #closest upcoming point that is outside of the nav radius
#            return min(i+step+1,len(path)-1)
#    
#    return i #no change in target index
	
def nextTarget(i,path,botPosition,navRadius): #arguments are index in path vector, path vector, current location of bot (3d), radius of bot for navigation
    print('current location: ',botPosition[:2])
    print('vector to target: ',path[i]-botPosition[:2])
    print('distance to target: ',np.linalg.norm(path[i]-botPosition[:2]))
    if i >= len(path)-1: #we are using the goal location
        return (len(path)-1) #use the goal location
    elif np.linalg.norm(path[i]-botPosition[:2]) > navRadius: #next point already outside of nav radius
        return i
    twoDist =0; #euclidean distance between current location and potential next target location
    while (twoDist <= navRadius) and (i <= len(path)-1):
        twoDist = np.linalg.norm(path[i]-botPosition[:2])
        i += 1
    return min(i,len(path)-1)

#testTarget = nextTarget(0,path,[0,0,0],neutralRadiusTotal)
#
#print(testTarget)
#print(path[testTarget])