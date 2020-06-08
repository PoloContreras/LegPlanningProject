# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import gym

from gym import envs

env = gym.make('MountainCar-v0')#Didn't I already do this for Project 2?
env.reset()

print('Initial state: ',env.reset())

print('State space: ',env.observation_space)

print('Action space: ',env.action_space)

print('observation low: ',env.observation_space.low)

print('observation high: ',env.observation_space.high)

for _ in range(1000):

    env.render()

    env.step(env.action_space.sample()) # take a random action

env.close()