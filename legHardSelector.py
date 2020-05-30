# -*- coding: utf-8 -*-
"""
Created on Sat May 23 21:33:43 2020

@author: user
"""

import numpy as np
#import cvxpy as cp
import math

radius = 1*math.sqrt(2)

circle_pos=np.array([1,1,0])

body_dir=np.array([0,1])

body_pos = circle_pos

point=np.array([2,-1,0])

# point and circle_pos should be a 3-dimensional vector
# projected point will be 2-dimensional
def circularProjection(circle_pos, circle_radius, point):
   rel_point = (point-circle_pos)[:2] # Only take the first two elements
   proj_point = rel_point / np.linalg.norm(rel_point)
   proj_point = circle_radius * proj_point # scale point
   proj_point = proj_point + circle_pos[:2] # shift the point back
   return proj_point

# Find the angle of a point with respect to the unit vector direction
# The position of the body must also be a parameter
# The body_dir and the point are both 2-dimensional
# The body_pos parameter is 3-dimensional
def positionToAngle(body_pos, body_dir, point):
   rel_point = point-body_pos[:2]
   norm_rel_point = rel_point / np.linalg.norm(rel_point)
   norm_dir = body_dir / np.linalg.norm(body_dir)
   print(norm_rel_point)
   # a cross b is positive if the angle is counter-clockwise
   sin_angle = norm_dir[0]*norm_rel_point[1]-norm_dir[1]*norm_rel_point[0]
   cos_angle = np.dot(norm_dir, norm_rel_point)
   angle = np.arccos(cos_angle) # Obtain the angle from dot product
   print(angle)
   if sin_angle >= 0:
      return angle
   elif sin_angle < 0:
      return 2*math.pi-angle



#Robot dimensions
#roboRadius = 0.2*math.sqrt(2) #radius of robot from geometric center to hip joint
#l1=0.2*math.sqrt(2) #length of first leg segment after hip joint
#l2=0.4*math.sqrt(2) #length of second seg segment (after ankle joint)

extremeHip = [math.radians(-30),math.radians(30)] #min/max angle achievable by hip joints
extremeAnkle = [math.radians(30),math.radians(70)] #min/max angle achievable by ankle joints
neutralAnkle = math.radians(40) #angle of ankle joint in neutral position

#base1 = 45
#base2 = 135
#base3 = -135
#base4 = -45 #angles of each hip at neutral position, from x axis

#neutralRadiusTotal = l2*math.cos(neutralAnkle)+l1+roboRadius #radius of pitch circle

#targetAngle = math.radians(315) #angle of target location with respect to x axis of robot, in degrees (single value for debugging)

#zNeutral = 0.4*math.sqrt(2)*math.sin(math.radians(neutralAnkle)) #Euclidean distance from geometric center to floor plane, in neutral position
#
#
#def zEffector(length1,length2,thetaHip,thetaAnkle):
#    z = -length2*math.sin(thetaAnkle)
#    return z
#
#def yEffector(length1,length2,thetaHip,thetaAnkle):
#    y = (length1+length2*math.cos(thetaAnkle))*math.cos(thetaHip)
#    return y
#
#def xEffector(length1,length2,thetaHip,thetaAnkle):
#    x = (length1+length2*math.cos(thetaAnkle))*math.sin(thetaHip)
#    return x

#def xBody(length1,length2,thetaHip,thetaAnkle,phiPitch,phiRoll):
#    x = xEffector(length1,length2,thetaHip,thetaAnkle)*math.sin(phiRoll)
#    return x
#
#def yBody(length1,length2,thetaHip,thetaAnkle,phiPitch,phiRoll):
#    y = yEffector(length1,length2,thetaHip,thetaAnkle)*math.sin(phiPitch)
#    return y
#
#def zBody(length1,length2,thetaHip,thetaAnkle,phiPitch,phiRoll):
#    z = zEffector(length1,length2,thetaHip,thetaAnkle)*math.cos(phiPitch)*math.cos(phiRoll)
#    return z

def numericSteps(targetRad):
    if targetRad >= 0 and targetRad < math.pi/2: #closest to leg 1
        return [(1,[extremeHip[0],neutralAnkle]),(3,[extremeHip[1],neutralAnkle])] #format: (choice,endpos) with endpos in degrees
    elif targetRad  >= math.pi/2 and targetRad < math.pi: #closest to leg 2
        return [(0,[extremeHip[1],neutralAnkle]),(2,[extremeHip[0],neutralAnkle])]
    elif targetRad >= math.pi and targetRad < 3*math.pi/2: #closest to leg 3
        return [(1,[extremeHip[1],neutralAnkle]),(3,[extremeHip[0],neutralAnkle])]
    else: #closest to leg 4 
        return [(0,[extremeHip[0],neutralAnkle]),(2,[extremeHip[1],neutralAnkle])]
    
#print(numericSteps(targetAngle)) #debug
    
projPoint = circularProjection(circle_pos,radius,point)
print(projPoint)
angleProj = positionToAngle(body_pos,body_dir,projPoint)
print(angleProj)
newLegs = numericSteps(angleProj)
print(newLegs)
    