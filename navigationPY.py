# -*- coding: utf-8 -*-
"""
Created on Sat May 23 21:33:43 2020

@author: user
"""

#generate data to read for navigation

import numpy as np
import cvxpy as cp

#DECLARATION OF VALUES FOR NAVIGATION

NavigatioN = 120 #total number of steps in navigation
pInitial = [0.1, 0.15] #Coordinates of initial position of agent
pFinal = [0.9,0.8] #coordinates of goal location

pObs = np.array([[0.6,0.4],[0.21,0.25]]) #positions of center of obstacles
rObs = [0.2,0.1] #radii of obstacles

obstacles = len(rObs) #total number of obstacles to dodge


#SOLVING TRAJECTORY AS CONVEX PROBLEM
pL = cp.Variable((NavigatioN,2)) #number of steps, number of dimensions, left trajectory
pR = cp.Variable((NavigatioN,2)) #number of steps, number of dimensions, right trajectory

gasolina = cp.expressions.expression.Expression #number 

gasolina = 0
for i in range(1,NavigatioN-1): #intermediate steps
    gasolina += cp.norm(pL[i-1,:]-2*pL[i,:]+pL[i+1,:],2) #calculation of 2-norms to be added for left trajectory
    gasolina += cp.norm(pR[i-1,:]-2*pR[i,:]+pR[i+1,:],2) #2-norms for right trajectory
         
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