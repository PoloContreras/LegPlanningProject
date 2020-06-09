# Convex Heuristics for Limb Placement
One of the contemporary challenges in robotic locomotion is the efficient calculation of the movements necessary both for limb collocation and navigation, such that the agent can react to its environment and effectively maneuver to a target location with minimal or no guidance from a human operator. 

This project casts this as two optimization problems: the problem of tracing a path between two locations while dodging an obstacle is cast as a cost minimization problem with convex approximations of nonconvex constraints, while the problem of calculating the movements necessary to move a limb to a certain position and take a step is also structured as a cost minimization problem with quasilinearized dynamics. The combination of these two approaches allows the agent to use principles of cost minimization to model its environment and thus react to its surroundings, using only details of its goal and environment geometry to inform its decisions.

## SW Packages required for running this repository
Follow the instructions for each of the listed packages to set up the environment needed to run this repo.

### Python 3.6+

### Mujoco 200
https://www.roboti.us/index.html

### Mujoco_py
https://github.com/openai/mujoco-py

### OpenAI Gym
https://gym.openai.com/docs/#installation

### Modern-Robotics
https://github.com/NxRLab/ModernRobotics/tree/master/packages/Python

### CVXPY
https://www.cvxpy.org/install/
**Note: ensure that the ECOS solver is installed and in the correct path.**

Windows users may find the following tutorial helpful as well:
https://medium.com/@sayanmndl21/install-openai-gym-with-box2d-and-mujoco-in-windows-10-e25ee9b5c1d5

## Running the simulations

To launch the main simulation execute the following command:
```
python robot_aug.py
```
