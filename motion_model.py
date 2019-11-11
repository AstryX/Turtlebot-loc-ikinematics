#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 10 22:00:13 2019

@author: tommy
"""

#!/usr/bin/env python2
import rospy
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from time import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import scipy.linalg
error =0.025
error_yaw = 0.05
yaw=pitch=roll=0.0
x=y=0
prev_x=prev_y=0

def get_rotation(msg):
    global yaw,pitch,roll
    global x,y
    orientation_p = msg.pose.pose.position
    orientation_q = msg.pose.pose.orientation
    pos = [orientation_p.x, orientation_p.y]

    x = pos[0]
    y = pos[1]
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    
def get_odom():
    rospy.Subscriber('/odom',Odometry,get_rotation)
 
def read_laser_scan_data():
    rospy.Subscriber('scan',LaserScan,laser_scan_callback)
 
def move_motor(fwd,ang):
    pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
    mc = Twist()
    mc.linear.x = fwd
    mc.angular.z = ang
    pub.publish(mc)
    print(fwd)


class Control(object):
    def __init__(self, x, v,maxspeed, minspeed, biasAcceleration=0, lin=True):
        self.lin= lin
        if self.lin ==False:
            self.x = yaw
            self.drift = 0.05
        else:
            self.x = x
            self.drift = 0.025
        self.v = v
        self.biasAcceleration = biasAcceleration
        
    def step(self, u, h):
        # Returns the updated position x and velocity v after one time step h
        a = self.biasAcceleration + u 
        print self.lin
        #self.x = self.x + h*self.v + 1./2.*a*h**2 # EOM for position
        self.v = self.v + h*a # EOM for velocity
        if self.v >maxspeed:
            self.v = maxspeed
        elif self.v < minspeed:
            self.v=minspeed
        if self.lin==True:
            move_motor(self.v,0)
        if self.lin==False:
            move_motor(0,self.v)
        
    def sense(self):
        global x,y,yaw
        get_odom()
        if self.lin==True:
            self.x=(x,y)
        else:
            self.x=yaw
        return self.x
    
def move_to_point(system, k_p, k_d, k_i, target):
    global x, y, yaw
    sys = system(0, 0)
        
    x_target = target # Set target position
    #  delta_x_target = 0.2  # Updating the target 
    
    h = 0.1 # dt
    x_old = sys.sense() # initial position of the system
    
    e_old = 0.2 # initial error
    e_i = 0 # initial intergral error
    
    #  x_list = []
    #  target_list = []
    reached = False
    while not reached:       
        # obtain current position of system  
        rospy.wait_for_message('/odom', Odometry)
        get_odom()
        state=sys.sense()
        # half-way update the target         
        
        # Implement the discretized PID loop equations
        ### START CODE HERE ### (~ 4 lines of code)
        if sys.lin==False:
            x_target2 = np.arctan2(x_target[1]-y,x_target[0]-x)
            print x_target2
            e = x_target2-state
            print 'ang error: '+str(e)#Calculate the error in position 
        if sys.lin==True:
            e = np.sqrt(np.power(target[0]-state[0],2)+np.power(target[1]-state[1],2))
        e_d = e/h # Calculate derivative of the error
        e_i = e_i+ (e*h) # Calculate integral of the error  
        u = (k_p*e) + (k_d*e_d) + (k_i*e_i)  # Calculate the output of the PID loop
        ### END CODE HERE ###
        print(e)
        if e < sys.drift:
            reached = True
            move_motor(0,0)
            print(e)
            print('finished')
        else:    
        # apply control effort, u (acceleration)
            sys.step(u, h)
        
        x_old = state # store previous position 
        e_old = e # store previous error 
        
        # store position, target and time for plotting reasons

# Tune the PID equations
### START CODE HERE ### (3 lines of code)
k_p = 0.08
k_d = 0.005
k_i = 0.01

k_p_ang = 0.3
k_i_ang = 0.05
k_d_ang = 0.01

maxspeed = 0.26
minspeed = 0.05

maxang = np.pi
minang = -np.pi

### END CODE HERE ###
if __name__ == '__main__':
    rospy.init_node('navigate',anonymous=True)
    rospy.Rate(10)
    rospy.wait_for_message('/odom', Odometry)	
    get_odom()
    systemLin = lambda x, v : Control(x,v,maxspeed,minspeed,0, True)
    systemAng = lambda yaw, v: Control(yaw,v,maxang,minang,0, False)
    point = (0.5,1)
    move_to_point(systemAng, k_p_ang, k_d_ang, k_i_ang, point)
    move_to_point(systemLin, k_p, k_d, k_i, point)
    
   
