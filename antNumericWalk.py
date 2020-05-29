# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import gym

from gym import envs

env = gym.make('Ant-v3')#to test different models change "CartPole-V0" to different models

env.reset()

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

def linearMotion(t): #input 0-1000, get 0 at 0 to 1 at 1000
    position = 0.001*t
    return position

for x in range(6000):

    env.render()

    paso = env.action_space.sample()
    #print(paso)
    
    initialStill = 1500
    forwardRight = initialStill+1000
    forwardLeft = forwardRight+1000
    pushOnGround = forwardLeft+1000
    rowYourLegs = pushOnGround+1000
    
    if x < initialStill: #move to neutral position
        posHip2 = 0
        posAnkle2 = -40*linearMotion(x*2/3)
        posHip4 = 0
        posAnkle4 = 40 *linearMotion(x*2/3)
        
        posAnkle3 = -40*linearMotion(x*2/3)
        posAnkle1 = 40*linearMotion(x*2/3)
        
    elif x < forwardRight:
        posHip2 = 0
        posAnkle2 = -40
        posHip4 = 30*linearMotion(x-initialStill)
        posAnkle4 = 35-5*quadraticMotion(x-initialStill)
        
        posAnkle3 = -40
        posAnkle1 = 40
    elif x < forwardLeft:
        posHip2 = -30*linearMotion(x-forwardRight)
        posAnkle2 = -35+5*quadraticMotion(x-forwardRight)
        posHip4 = 30
        posAnkle4 = 40
        
        posAnkle3 = -40
        posAnkle1 = 40
    elif x < pushOnGround:
        posHip2 = -30
        posAnkle2 = -30*linearMotion(x-forwardLeft)-40
        posHip4 = 30
        posAnkle4 = 30*linearMotion(x-forwardLeft)+40
        
        posAnkle3 = -40
        posAnkle1 = 40
    elif x < rowYourLegs: #merrily
        posHip2 = 30*linearMotion(x-pushOnGround)-30
        posAnkle2 = -70
        posHip4 = -30*linearMotion(x-pushOnGround)+30
        posAnkle4 = 70
        
        posAnkle3 = -40
        posAnkle1 = 40
    else: #return to neutral position
        posHip2 = 0
        posAnkle2 = -70 + 30*linearMotion(x-rowYourLegs)
        posHip4 = 0
        posAnkle4 = 70 - 30*linearMotion(x-rowYourLegs)
        posAnkle3 = -40
        posAnkle1 = 40
    
    state = env.data.qpos
    
    centerMass = env.data.subtree_com
    print(centerMass)
    
#    inertiaMatrix = mj.data.qm
#    print(inertiaMatrix)
    
#    print(pos)
#    print(sum([state[3],state[4],state[5]]))
    
#    if minHeight > state[2]:
#        minHeight = state[2]
#        
#    if maxHeight < state[2] and x > 350:
#        maxHeight = state[2]
    
    paso = [posHip4,posAnkle4,0,posAnkle1,posHip2,posAnkle2,0,posAnkle3]
#    paso =[20]*8
    
    env.step(paso) # take a random action

#print('Min height: ',minHeight)
#print('Max height: ',maxHeight)

env.close()