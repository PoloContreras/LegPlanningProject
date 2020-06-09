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
pInitial = np.array([0, 0]) #Coordinates of initial position of agent
pFinal = np.array([30,8]) #coordinates of goal location

pObs = np.array([[5,2],[12,6],[12,1],[20,3],[26,6],[26,1]]) #positions of center of obstacles
rObs = np.array([[2.06],[1.78],[1.78],[3.81],[1.66],[1.56]]) #radii of obstacles

obstacles = len(rObs) #total number of obstacles to dodge

pt = np.ones((NavigatioN,2)) #steps in navigation, 2D space

for i in range(1,NavigatioN-1):
	pt[i,:] = pInitial+(((pFinal-pInitial)/NavigatioN)*i) #linear interpolation for lowest cost path with no obstacles

#print(pt)
	
diff = math.inf #initialize to unreasonable value to overwrite in loop
epsilon = 0.01 #tolerance for convergence of the solution

while abs(diff) > epsilon:
	#EXTRANEOUS CALCULATION OF OBJECTIVE VALUE
	prevGas = 0
	for i in range(1,NavigatioN-1):
		prevGas = prevGas + np.linalg.norm(pt[i-1,:]-2*pt[i,:]+pt[i+1,:])**2
	
	print('prev: ',prevGas)
	
	#SOLVING TRAJECTORY AS CONVEX PROBLEM
	pL = cp.Variable((NavigatioN,2)) #number of dimensions, number of steps, details of trajectory
	#pR = cp.Variable((NavigatioN,2)) #number of steps, number of dimensions, right trajectory
	
	gasolina = cp.expressions.expression.Expression #to hold value of objective function
#	distance = cp.expressions.expression.Expression #to hold a given distance between a step and obstacle
#	divergence = cp.expressions.expression.Expression #to hold the difference between the variable value and previous calculation for a step
	
	gasolina = 0
	for i in range(1,NavigatioN-1): #intermediate steps
	    gasolina += cp.norm(pL[i-1,:]-2*pL[i,:]+pL[i+1,:],2)**2 #calculation of squared 2-norms to be added for objective function
	    #gasolina += cp.norm(pR[i-1,:]-2*pR[i,:]+pR[i+1,:],2)**2 #squared 2-norms for right trajectory
	         
	objectNav = cp.Minimize(gasolina)
	
	constrNav = [] #initialize constraints list
	 
	constrNav += [pL[0,:] == pInitial] #initial and final positions
	constrNav += [pL[NavigatioN-1,:] == pFinal]
#	constrNav += [pR[0,:] == pInitial] #right trajectory
#	constrNav += [pR[NavigatioN-1,:] == pFinal]
	
	    
	for o in range(0,obstacles): #constraints for each obstacle
	    for i in range(1,NavigatioN-1): #apply constraints to mutable steps
#	        distance = pt[i,:] - pObs[o,:]
#	        print(distance)
#	        divergence = pL[i,:]-pt[i,:]
#	        print(divergence)
#	        constrNav += [rObs[o] - cp.norm(distance) - (distance/cp.norm(distance))*np.transpose(pL[i,:]-pt[i,:]) <= 0]
#	        print(pt[i,:] - pObs[o,:])
#	        print(np.transpose(pt[i,:] - pObs[o,:]))
	        constrNav += [rObs[o] - cp.norm((pt[i,:] - pObs[o,:])) - np.dot(((pt[i,:] - pObs[o,:])/cp.norm((pt[i,:] - pObs[o,:]))),(pL[i,:]-pt[i,:])) <= 0]
	
	problem = cp.Problem(objectNav,constrNav)
	optval = problem.solve(solver=cp.ECOS)
	#END OF CONVEX OPTIMIZATION PART
	
	print('opt :',optval) #monitor
	pt = copy.deepcopy(pL.value) #for use in following iteration
	diff = prevGas - optval #for checking convergence via while loop
	
	
#print(objectNav)
print('final opt val: ',optval)
#print(pL.value)
#print(pR.value)

#dieseL = 0
#dieseR = 0
#for g in range(1,NavigatioN-1): #selecting which path to take
#    dieseL += np.linalg.norm(pL.value[g-1,:]-2*pL.value[g,:]+pL.value[g+1,:],2)**2
#    dieseR += np.linalg.norm(pR.value[g-1,:]-2*pR.value[g,:]+pR.value[g+1,:],2)**2
   
path = copy.deepcopy(pL.value) #once more with feeling 
#if dieseL <= dieseR:
#   path = copy.deepcopy(pL.value)
#    print('left')
#else:
#    path = copy.deepcopy(pR.value)
#    print('right')
print(path)
    
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