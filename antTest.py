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

maxHeight = 0

minHeight= 1

for x in range(1000):

    env.render()

    paso = env.action_space.sample()
    #print(paso)
    
    pos = -0.000008*x*x + 0.008*x-1
    
    pos = pos*35+35
    
    state = env.data.qpos
    
#    print(pos)
#    print(sum([state[3],state[4],state[5]]))
    
    if minHeight > state[2]:
        minHeight = state[2]
        
    if maxHeight < state[2] and x > 350:
        maxHeight = state[2]
    
    paso = [0,pos,0,pos,0,-pos,0,-pos]
#    paso =[20]*8
    
    env.step(paso) # take a random action

print('Min height: ',minHeight)
print('Max height: ',maxHeight)

env.close()