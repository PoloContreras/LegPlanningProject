# -*- coding: utf-8 -*-
"""
Created on Fri May 22 21:14:27 2020

@author: user
"""

import numpy as np
import modern_robotics as mr
import math
import cvxpy as cp
import matplotlib.pyplot as plt

import gym
from gym import envs
import modern_robotics as mr

env = gym.make('Ant-v3')

body_frames = env.data.body_xmat
body_pos = env.data.body_xpos

end_pos = np.array([30, 8])

GoalReached = False

env.reset()
env.render()

# We can literally put all of the following code in one big while loop
while(not GoalReached):


	# Front legs at start
	#body_frames = env.data.ximat
	#body_pos = env.data.xipos

	#R_main = body_frames[1].reshape(3, 3)
	#T_main = np.r_[np.c_[R_main, body_pos[1]], np.array([0, 0, 0, 1]).reshape(1,4)]
	#T_main_inv = np.linalg.inv(T_main)

	#R_hip_1 = body_frames[1].reshape(3, 3)

	# Body frames with respect to the joint positions
	body_frames = env.data.body_xmat
	body_pos = env.data.body_xpos

	R_main = body_frames[1].reshape(3, 3)
	T_main = np.r_[np.c_[R_main, body_pos[1]], np.array([0, 0, 0, 1]).reshape(1,4)]
	T_main_inv = np.linalg.inv(T_main)

	# All of the body frames must be relative to one another so we must multiply by inverses
	# AB = C => B = A_inv*C Where B is the relative matrix and assuming invertibility

	# map_to_leg maps the chosen leg (0-3) to the corresponding position in body frames list
	map_to_leg = [3, 6, 9, 12]
	choice = 0
	chosen_leg = map_to_leg[choice]
	R_hip = body_frames[chosen_leg].reshape(3, 3)
	T_hip = np.r_[np.c_[R_hip, body_pos[chosen_leg]], np.array([0, 0, 0, 1]).reshape(1,4)]
	R_ankle = body_frames[chosen_leg+1].reshape(3, 3)
	T_hip_inv = np.linalg.inv(T_hip)
	T_ankle = T_hip_inv@np.r_[np.c_[R_ankle, body_pos[chosen_leg+1]], np.array([0, 0, 0, 1]).reshape(1, 4)]

	# Create the list of body frame matrices
	Mlist = np.array([T_main, T_hip, T_ankle])

	# For cylindar off hip joint: Density is 5. Radius is 0.04. Length is 0.2*sqrt(2)
	# For cylindar off ankle joint: Density is 5. Radius is 0.04. Length is 0.4*sqrt(2)
	# Mass is Density*pi*(Radius)^2*Length
	# Inertia for cylindar:
	# I_xx = m(3r^2+h^2)/12
	# I_yy = m(3r^2+h^2)/12
	# I_zz = mr^2/2

	rho_hip = 5
	r_hip = 0.04
	h_hip = 0.2*math.sqrt(2)
	m_hip = rho_hip*(3*r_hip**2+h_hip**2)/12
	rho_ankle = 5
	r_ankle = 0.04
	h_ankle = 0.4*math.sqrt(2)
	m_ankle = rho_ankle*(3*r_ankle**2+h_ankle**2)/12

	I_xx_hip = m_hip*(3*r_hip**2+h_hip**2)/12
	I_yy_hip = m_hip*(3*r_hip**2+h_hip**2)/12
	I_zz_hip = m_hip*r_hip**2/2

	I_xx_ankle = m_ankle*(3*r_ankle**2+h_ankle**2)/12
	I_yy_ankle = m_ankle*(3*r_ankle**2+h_ankle**2)/12
	I_zz_ankle = m_ankle*r_ankle**2/2

	I_hip = np.diag([I_xx_hip, I_yy_hip, I_zz_hip, m_hip, m_hip, m_hip])
	I_ankle = np.diag([I_xx_ankle, I_yy_ankle, I_zz_ankle, m_ankle, m_ankle, m_ankle])

	# Construct the list of inertia matrices
	Glist = np.array([I_hip, I_ankle])

	hip_axis = 0
	ankle_axis = 0

	# Use XML information to calculate joint orientation
	if choice == 0:
		hip_axis = np.linalg.inv(R_main) @ R_hip @ np.array([0, 0, 1])
		ankle_axis = np.linalg.inv(R_main) @ R_hip @ R_ankle @ np.array([-1, 1, 0])
	elif choice == 1:
		hip_axis = np.linalg.inv(R_main) @ R_hip @ np.array([0, 0, 1])
		ankle_axis = np.linalg.inv(R_main) @ R_hip @ R_ankle @ np.array([1, 1, 0])
	elif choice == 2:
		hip_axis = np.linalg.inv(R_main) @ R_hip @ np.array([0, 0, 1])
		ankle_axis = np.linalg.inv(R_main) @ R_hip @ R_ankle @ np.array([-1, 1, 0])
	elif choice == 3:
		hip_axis = np.linalg.inv(R_main) @ R_hip @ np.array([0, 0, 1])
		ankle_axis = Rnp.linalg.inv(R_main) @ _hip @ R_ankle @ np.array([1, 1, 0])

	# Below is necessary because we need q w.r.t. our fixed frame (the main body frame)
	q_hip = (T_main_inv @ np.r_[body_pos[chosen_leg], [1]])[:3]
	q_ankle = (T_main_inv @ np.r_[body_pos[chosen_leg+1], [1]])[:3]

	# Construct the list of screw 6-vectors
	S_hip = np.r_[hip_axis, np.negative(np.cross(hip_axis, q_hip))]
	S_ankle = np.r_[ankle_axis, np.negative(np.cross(ankle_axis, q_ankle))]

	Slist = np.array([S_hip, S_ankle]).T

	'''
	T_main_inv = np.linalg.inv(T_main)
	hip_1 = (0,0,1)
	ankle_1 = (-1,1,0)
	hip_2 = (0,0,1)
	ankle_2 = (1,1,0)
	# Back legs at start
	hip_3 = (0,0,1)
	ankle_3 = (-1,1,0)
	hip_4 = (0,0,1)
	ankle_4 = (1,1,0)
	
	print(env.data.ximat)
	s_hat = env.data.ximat
	np.array()
	
	G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
	G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
	G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
	Glist = np.array([G1, G2, G3])
	Mlist = np.array([M01, M12, M23, M34])
	Slist = np.array([[1, 0, 1,      0, 1,     0],
					  [0, 1, 0, -0.089, 0,     0],
					  [0, 1, 0, -0.089, 0, 0.425]]).T
	
	mr.MassMatrix(thetalist, Mlist, Glist, Slist)
	'''
	"""Computes the Coriolis and centripetal terms in the inverse dynamics of
	an open chain robot
	:param thetalist: A list of joint variables,
	:param dthetalist: A list of joint rates,
	:param Mlist: List of link frames i relative to i-1 at the home position,
	:param Glist: Spatial inertia matrices Gi of the links,
	:param Slist: Screw axes Si of the joints in a space frame, in the format
				  of a matrix with axes as the columns.
	:return: The vector c(thetalist,dthetalist) of Coriolis and centripetal
			 terms for a given thetalist and dthetalist.
	This function calls InverseDynamics with g = 0, Ftip = 0, and
	ddthetalist = 0.
	Example Input (3 Link Robot):
		thetalist = np.array([0.1, 0.1, 0.1])
		dthetalist = np.array([0.1, 0.2, 0.3])
		M01 = np.array([[1, 0, 0,        0],
						[0, 1, 0,        0],
						[0, 0, 1, 0.089159],
						[0, 0, 0,        1]])
		M12 = np.array([[ 0, 0, 1,    0.28],
						[ 0, 1, 0, 0.13585],
						[-1, 0, 0,       0],
						[ 0, 0, 0,       1]])
		M23 = np.array([[1, 0, 0,       0],
						[0, 1, 0, -0.1197],
						[0, 0, 1,   0.395],
						[0, 0, 0,       1]])
		M34 = np.array([[1, 0, 0,       0],
						[0, 1, 0,       0],
						[0, 0, 1, 0.14225],
						[0, 0, 0,       1]])
		G1 = np.diag([0.010267, 0.010267, 0.00666, 3.7, 3.7, 3.7])
		G2 = np.diag([0.22689, 0.22689, 0.0151074, 8.393, 8.393, 8.393])
		G3 = np.diag([0.0494433, 0.0494433, 0.004095, 2.275, 2.275, 2.275])
		Glist = np.array([G1, G2, G3])
		Mlist = np.array([M01, M12, M23, M34])
		Slist = np.array([[1, 0, 1,      0, 1,     0],
						  [0, 1, 0, -0.089, 0,     0],
						  [0, 1, 0, -0.089, 0, 0.425]]).T
	Output:
		np.array([0.26453118, -0.05505157, -0.00689132])
	"""
	m1 = 1  # masses
	m2 = 5
	l1 = 1  # link lengths
	l2 = 1

	N = 40  # steps
	T = 10  # time
	h = T / N  # discretization of time

	# Hard-coded for the testing phase (in radians).
	startpos = [0, 0]
	endpos = [0.2618, 0.5234]  # THESE WERE COLUMN VECTORS IN THE MATLAB CODE

	taumax = 1.1  # maximum torque
	alpha = 0.1
	betasucc = 1.1
	betafail = 0.5
	rhoinit = 90 * math.pi / 180
	lambdalambda = 2;
	Kmax = 40

	# Set the initial trajectory to linearly interpolate the two points
	ot = np.c_[startpos, [np.linspace(startpos[0], endpos[0], N), np.linspace(startpos[1], endpos[1], N)], endpos]
	otdot = np.zeros([2, N+2])
	otddot = np.zeros([2, N+2])

	for t in range(1, N + 1):
		otdot[0, t] = (ot[0, t]-ot[0, t-1])/h
		otdot[1, t] = (ot[1, t]-ot[1, t-1])/h
		otddot[0, t] = (ot[0,t+1] - 2*ot[0, t]+ot[0, t-1])/(h*h)
		otddot[1, t] = (ot[1, t+1]-2*ot[1, t]+ot[1, t-1])/(h*h)

	etas = np.zeros(Kmax)
	etahats = np.zeros(Kmax)
	Js = np.zeros(Kmax)
	phis = np.zeros(Kmax)
	rhos = np.zeros(Kmax)
	rhos[0] = rhoinit
	phihats = np.zeros(Kmax)
	pdec = np.zeros(Kmax)
	adec = np.zeros(Kmax)

	# Calculate the initial value of phi
	eta = np.zeros([2, N])
	for t in range(1, N + 1):  # Dynamics equations.
		t1 = ot[0, t]
		t2 = ot[1, t]
		t1dot = otdot[0, t]
		t2dot = otdot[1, t]

		# Theta values begin at [0,0,0] which corresponds to the middle
		# of the given angle range.
		thetalist = np.array([t1, t2])
		# Derivatives are approximations for now
		dthetalist = np.array([t1dot, t2dot])

		M = mr.MassMatrix(thetalist, Mlist, Glist, Slist)
		W = mr.VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)

		eta[:, t - 1] = np.negative(sum([M@otddot[:, t], W@otdot[:, t]]))

	oldphi = 0 + lambdalambda * np.sum(np.abs(eta))

	i_count = 0

	for i in range(0, Kmax):
		nt = cp.Variable((2, N+2))
		ntdot = cp.Variable((2, N+2))
		ntddot = cp.Variable((2, N+2))
		tau = cp.Variable((2, N+2)) # torque
		etahat = cp.Variable((2, N))
		#etahat = cp.expressions.expression.Expression((2, N))

		constr = [] # Initialize constraints list

		# initial and final conditions
		constr += [nt[:, 0] == startpos, nt[:, 1] == startpos]
		constr += [nt[:, N] == endpos, nt[:, N+1] == endpos]
		# limb starts and ends at zero velocity and torque
		constr += [tau[:, 0] == 0, tau[:, N+1] == 0]

		for t in range(1, N+1):
			# Consistency of first derivatives.
			constr += [ntdot[:, t] == (nt[:, t+1]-nt[:, t-1])/(2*h)]

			# Consistency of double derivatives.
			constr += [ntddot[:, t] == (nt[:, t+1]-2*nt[:, t]+nt[:, t-1])/(h*h)]

			# Dynamics equations. CONVERT TO PYTHON
			t1 = ot[0, t]
			t2 = ot[1, t]
			t1dot = otdot[0, t]
			t2dot = otdot[1, t]

			thetalist = np.array([t1, t2])
			dthetalist = np.array([t1dot, t2dot])

			M = mr.MassMatrix(thetalist, Mlist, Glist, Slist)
			W = mr.VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)

			constr += [etahat[:, t-1] == tau[:, t]-M@ntddot[:, t]-W@ntdot[:, t]]

		# Trust region constraints.
		constr += [cp.abs(nt[:]-ot[:]) <= rhos[i]]

		# Torque limit.
		constr += [cp.abs(tau[:]) <= taumax]

		# Second arm can't fold back on first.
		constr += [nt[1, :] <= math.pi, nt[1, :] >= -1*math.pi]

		# Establish objective function
		objective = cp.Minimize(h*cp.sum_squares(tau[:])+lambdalambda*cp.sum(cp.abs(etahat[:])))

		problem = cp.Problem(objective, constr)
		optval = problem.solve(solver=cp.ECOS) # Solve...That...Problem!

		# Calculate actual torque violations.
		eta = np.zeros((2, N))
		for t in range(1, N+1):
			# Dynamics equations. CONVERT TO PYTHON
			t1 = nt.value[0, t]
			t2 = nt.value[1, t]
			t1dot = ntdot.value[0, t]
			t2dot = ntdot.value[1, t]

			thetalist = np.array([t1, t2])
			dthetalist = np.array([t1dot, t2dot])

			M = mr.MassMatrix(thetalist, Mlist, Glist, Slist)
			W = mr.VelQuadraticForces(thetalist, dthetalist, Mlist, Glist, Slist)

			eta[:, t-1] = sum([tau.value[:, t], np.negative(sum([M@ntddot.value[:, t], W@ntdot.value[:, t]]))])

		etahats[i] = np.sum(np.abs(etahat.value[:]))
		etas[i] = np.sum(np.abs(eta[:]))
		Js[i] = h*np.sum(tau.value**2)  # CONVERT TO CVXPY

		phihats[i] = Js[i]+lambdalambda*etahats[i]
		phis[i] = Js[i]+lambdalambda*etas[i]

		deltahat = oldphi-phihats[i]
		delta = oldphi-phis[i]
		oldphi = phis[i]

		pdec[i] = deltahat
		adec[i] = delta

		if delta <= alpha*deltahat:
			if i != Kmax-1:
				rhos[i + 1] = betafail*rhos[i]
				i_count += 1

			# Undo recording of the last step.
			if i != 0:
				Js[i] = Js[i-1]
				phis[i] = phis[i-1]
				etas[i] = etas[i-1]
				etahats[i] = etahats[i-1]
				oldphi = phis[i]
		else:
			if i != Kmax-1:
				rhos[i+1] = betasucc * rhos[i]

			#print(tau.value)
			# Propagate the system.
			ot = nt.value
			otdot = ntdot.value

	# See if we got close enough
	GoalReached = True if np.norm(body_pos[1] - end_pos) <= 1 else False

	k = 0
	theta = ot * (180 / math.pi)

	# Advance the simulation
	for i in range(2000):
		env.render()

		paso = [0, 30, theta[0, (i - 1000) // 5], 30 + theta[1, (i - 1000) // 5], 0, -30, 0, -30]

env.close()


# Below is an educated guess on timings
# timestep = 0.01 seconds per frame
# total_frames = 5*(total_steps)
# T = 10 seconds for when we're giving inputs
# 5 frames per step
# UNDER ASSUMPTION THAT EACH FRAME IS 0.01 SECONDS
# 100 frames per second. 1000 frames per 10 seconds
# 200 steps
# N = 40 we will have 4 inputs per second (one every 25 frames)


# print(theta)
# print(i_count)
# body_frames = env.data.body_xmat
# body_pos = env.data.body_xpos
# print("body_frames = ", body_frames)
# print("body_pos = ", body_pos)
def computeBodyRotation():
	xAxis = np.array([1,0])
	torsoVec = env.data.body_xmat[1].reshape(3, 3)[:2,0]
	torsoVec /= np.linalg.norm(torsoVec)
	sin_angle = xAxis[0] * torsoVec[1] - xAxis[1] * torsoVec[0]
	angle = np.arccos(np.dot(xAxis, torsoVec))

	if sin_angle >= 0:
		rotation = angle
	elif sin_angle < 0:
		rotation = 2*math.pi-angle

	return rotation #both vectors normalized

rotation = computeBodyRotation()
'''
GoalReached = False
while(not GoalReached):
	for i in range(2000):
		env.render()
		# Each specific angle should be held for five steps to correspond
		# with the time-frame
		prev = 0
		if i <=1000:
			env.step([0, 30, 0, 30, 0, -30, 0, -30])
		if i>1000 and i < 1000+5*N-1:
			paso = [0, 30,  theta[0, (i-1000)//5], 30+theta[1, (i-1000)//5],0, -30, 0, -30]
			if int((i - 1000) // 5)>prev+0.5:
				prev = int((i - 1000)//5)
				# print(paso)
				k+=1
				# print(prev)
				# print(k)
			env.step(paso)
			continue
		if i > 1000+5*N-1:
			env.step([0, 30, theta[0, 40], 30+theta[1, 40], 0, -30, 0, -30])
	# Compute location
	rotation = computeBodyRotation()

	# print("body_frames = ", body_frames)
	# print("body_pos = ", body_pos)

	# Check if goal done
	GoalReached = True
'''

