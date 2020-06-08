# -*- coding: utf-8 -*-
"""
Spyder Editor

This is a temporary script file.
"""

import gym

from gym import envs
print(envs.registry)

env = gym.make('FetchReach-v1')#to test different models change "CartPole-V0" to different models

env.reset()

for _ in range(1000):

    env.render()

    env.step(env.action_space.sample()) # take a random action

env.close()