# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import gym

from gym import envs



#print('Initial state: ',env.reset())
#
#print('State space: ',env.observation_space)
#
#print('Action space: ',env.action_space)
#
#print('observation low: ',env.observation_space.low)
#
#print('observation high: ',env.observation_space.high)
#
#maxHeight = 0
#
#minHeight= 1

def quadraticMotion(t): #input 0-1000, get -1 at 0 to 1 at 500 to -1 at 1000
    posicion = -0.000008*t*t + 0.008*t-1
    return posicion

def linearMotion(t,phaseTimer): #input 0-1000, get 0 at 0 to 1 at 1000
    position = t/phaseTimer #relative position between 0 and 1
    return position

def numericWalk(phaseTimer,x,new_legs,avance):
#    choiceMapped = [3,0,1,2]
    pushOnGround = phaseTimer
    rowYourLegs = pushOnGround+phaseTimer
    
    choice_1,theta_end_1 = new_legs[0]
    choice_2,theta_end_2 = new_legs[1]
	
#    choice_1 = choiceMapped[choice_1]
#    choice_2 = choiceMapped[choice_2]
    
#    if choice_1 == 0 or choice_1 == 3: #you're moving the diagonal across from it as well, so only one choice matters
#        sign =1
#    else:
#        sign = -1
    
    tobillos = [0,70,0,70,0,-70,0,-70] #maximum extent of ankle joints
    
    
    if x < pushOnGround:
        #avance[2*choiceMapped[choice_1]] = avance[whatevs] #hips stay in place
        #avance[2*choiceMapped[choice_2]] = avance[whatevs]
        avance[2*choice_1+1] += (tobillos[2*choice_1+1]-avance[2*choice_1+1])*linearMotion(x,phaseTimer) #ankles push down
        avance[2*choice_2+1] += (tobillos[2*choice_2+1]-avance[2*choice_2+1])*linearMotion(x,phaseTimer)    
    elif x < rowYourLegs: #merrily
        x -= pushOnGround #recalibrating for linearMotion
        avance[2*choice_1] *= (1-linearMotion(x,phaseTimer))
        avance[2*choice_2] *= (1-linearMotion(x,phaseTimer))
        avance[2*choice_1+1] = tobillos[2*choice_1+1] #ankles down
        avance[2*choice_2+1] = tobillos[2*choice_2+1]    
    else: #return to neutral position
        x -= rowYourLegs
        avance[2*choice_1] = 0
        avance[2*choice_2] = 0
        avance[2*choice_1+1] += (tobillos[2*choice_1+1]-avance[2*choice_1+1])*(1-linearMotion(x,phaseTimer)) #ankles return
        avance[2*choice_2+1] += (tobillos[2*choice_2+1]-avance[2*choice_2+1])*(1-linearMotion(x,phaseTimer))    
#        if x == 250:
#            print('avance: ',(avance[2*choice_1+1]))
		
#    inertiaMatrix = mj.data.qm
#    print(inertiaMatrix)
    
#    print(pos)
#    print(sum([state[3],state[4],state[5]]))
    
#    if minHeight > state[2]:
#        minHeight = state[2]
#        
#    if maxHeight < state[2] and x > 350:
#        maxHeight = state[2]
    
    return avance
#    paso =[20]*8
    
#    env.step(paso) # take a random action

#print('Min height: ',minHeight)
#print('Max height: ',maxHeight)