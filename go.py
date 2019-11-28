#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 26 15:18:53 2019

@author: tommy
"""
import motion2
import rospy
goals = [(3.95,0.31,-90*np.pi/180), # centre point of button h: -0.12245 (relative to arm) 
	(0.63,0.55,90*np.pi/180), # +12.5x, +27y,  
	(0.5,2.31,90*np.pi/180), #
	(2.81,1.17, 0*np.pi/180),
	(2.72,2.76, 0*np.pi/180)]
N=10
poses=[(1.07,0,0), (0.3,0,0)]
prev_parts = np.zeros(N,3)
state = 

if __name__ == "__main__":
    rospy.init_node('navigate',anonymous=True)
    rospy.Rate(10)
    for pose in poses:
        global prev_parts, state 
        prev_parts, state= motion2.move_pose(pose,prev_parts,state)
        
    print 'all_finished'

'''
from aggron:

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 26 15:18:53 2019

@author: tommy
"""
import motion2
import rospy
import numpy as np
import move_arm
N=10
poses=[(1.15,0,0),(0.2,0.2,90*np.pi/180),(1.5,0,90*np.pi/180)]
prev_parts = np.zeros((N,3))
state = (0,0,0)

if __name__ == "__main__":
    rospy.init_node('navigate',anonymous=True)
    rospy.Rate(10)
    global prev_parts, state 
    prev_parts, state= motion2.move_pose(poses[0],prev_parts,state)
    move_arm.complete_arm_task("press",0,[0.2, -0.12245], False)
    prev_parts, state = motion2.move_pose(poses[1],prev_parts,state)
    prev_parts, state = motion2.move_pose(poses[2],prev_parts,state)
	
	
        
    print 'all_finished'

'''
