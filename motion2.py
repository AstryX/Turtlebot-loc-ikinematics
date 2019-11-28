#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 10 22:00:13 2019

@author: tommy
"""

#!/usr/bin/env python2
import rospy
import pf
import numpy as np
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from time import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler

error =0.025
error_yaw = 0.075
yaw=pitch=roll=0.0
x=y=0
prev_x=prev_y=prev_yaw=0
x_init = 4.25-0.42
y_init = 2.15-0.35
N=10
previous_parts=np.zeros((N,3))
beams = []
valid_angs = range(0,359,45)
old_state = (0,0,0)


def tf(control):
    x_w = x_init-control[1]
    y_w = y_init - control[0]
    yaw_w = (-90*np.pi/180)-control[2]
    if yaw_w < 0:
        yaw_w = yaw_w%(-2*np.pi)
        if yaw_w < -np.pi:
            yaw_w+=2*np.pi
    else:
        yaw_w = yaw_w = yaw_w%(2*np.pi)
        if yaw_w > np.pi:
            yaw_w-=2*np.pi
    
    return x_w,y_w,yaw_w

def get_rotation(msg):
    timeo=time()
    global yaw,pitch,roll
    global x,y
    orientation_p = msg.pose.pose.position
    orientation_q = msg.pose.pose.orientation
    pos = [orientation_p.x, orientation_p.y]

    x = pos[0]
    y = pos[1]
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll,pitch,yaw) = euler_from_quaternion(orientation_list)
    curr = time()
    #print 'time in odom: ' +str(curr-timeo)
    
def get_odom():
    rospy.Subscriber('/odom',Odometry,get_rotation)
    
def laser_scan_callback(data):
    times = time()
    global beams 
    beams = []
    for i in valid_angs:
        if data.ranges[i]>0.1:
            
            beams.append(data.ranges[i])
        else:
            beams.append(3.5)
   
    end = time()
    #print 'time laser scan = ' + str(end-times)
def read_laser_scan_data():
    rospy.Subscriber('scan',LaserScan,laser_scan_callback)
 
def move_motor(fwd,ang):
    pub = rospy.Publisher('cmd_vel',Twist,queue_size = 10)
    mc = Twist()
    mc.linear.x = fwd
    mc.angular.z = ang
    pub.publish(mc)
    print('lin spd: '+ str(fwd))
    print('ang spd: ' + str(ang))


class Control(object):
    def __init__(self, x, v,maxspeed, minspeed, biasAcceleration=0, lin=True, final=False):
        self.lin= lin
        self.final = final 
        if self.lin ==False:
            self.x = yaw
            self.drift = 0.05
        else:
            self.x = x
            self.drift = 0.05
        self.v = v
        self.biasAcceleration = biasAcceleration
        
    def step(self, u, h):
        # Returns the updated position x and velocity v after one time step h
        a = self.biasAcceleration + u 
        print 'accel = '+str(a) 
        print ('is linear = ' +str(self.lin))
        #self.x = self.x + h*self.v + 1./2.*a*h**2 # EOM for position
        self.v = self.v + h*a # EOM for velocity
        if abs(self.v) >maxspeed:
            if self.v > 0.:
                self.v = maxspeed
            if self.v < 0.:
                self.v = -maxspeed
        elif abs(self.v) < minspeed:
            if self.v > 0:
                self.v=minspeed
            if self.v < 0:
                self.v = -minspeed
        if self.lin==True:
            move_motor(self.v,0)
        if self.lin==False:
            move_motor(0,self.v)
        
    def sense(self):
        global x,y,yaw
        #place particle filter here.
        x=x
	y=y 
	yaw=yaw
        cont = ((prev_x,prev_y,prev_yaw),(x,y,yaw))
        parts=pf.move_particles(cont,previous_parts,beams)
        state_est = np.mean(parts, axis=0)
        est_x=state_est[0]
        est_y=state_est[1]
        est_yaw = state_est[2]
        if est_yaw > 0:
            est_yaw = est_yaw%(-2*np.pi)
            if est_yaw < -np.pi:
                est_yaw+=2*np.pi
        else:
            est_yaw = est_yaw%(2*np.pi)
            if est_yaw > np.pi:
                est_yaw-=2*np.pi
        if self.lin==True:
            self.x=(est_x,est_y)
            x= est_x
            y=est_y
            yaw = est_yaw
        else:
            self.x=est_yaw
            yaw = est_yaw
            x = est_x
            y = est_y
        print 'pf state is: ' +str(state_est)    
        print 'est state is: '+str(x)+', '+str(y)+', '+str(yaw)   
        global previous_parts 
        
        previous_parts= parts
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
        rospy.wait_for_message('scan',LaserScan)
        s_time = time()
        state=sys.sense()
        curr = time()
        print 'time taken by pf: '+str(curr-s_time)+ ' seconds'
        # half-way update the target         
        
        # Implement the discretized PID loop equations
        ### START CODE HERE ### (~ 4 lines of code)
        if sys.lin==False and sys.final==False:
            x_target2 = np.arctan2(x_target[1]-y,x_target[0]-x)
            print 'ang_target = ' +str(x_target2)
            e = x_target2-state
            print 'ang error: '+str(e)#Calculate the error in position 
        if sys.lin==True:
            e = np.sqrt(np.power(target[0]-state[0],2)+np.power(target[1]-state[1],2))
        if sys.lin==False and sys.final ==True:
            e = target[2]-yaw
        e_d = e/h # Calculate derivative of the error
        e_i = e_i+ (e*h) # Calculate integral of the error  
        u = (k_p*e) + (k_d*e_d) + (k_i*e_i)  # Calculate the output of the PID loop
        ### END CODE HERE ###
        print 'error is '+str(e)
        if abs(e) < sys.drift:
            reached = True
            move_motor(0,0)
            print 'final error is ' + str(e)
            print('finished')
        else:    
        # apply control effort, u (acceleration)
            sys.step(u, h)
        
        x_old = state # store previous position 
        global prev_x, prev_y, prev_yaw
        if sys.lin:
            prev_x=x_old[0]
            prev_y = x_old[1]
        else:
            prev_yaw = x_old
        e_old = e # store previous error 
        
    return x_old
        
        # store position, target and time for plotting reasons

# Tune the PID equations
### START CODE HERE ### (3 lines of code)
k_p = 0.08
k_d = 0.005
k_i = 0.01

k_p_ang = 0.3
k_i_ang = 0.05
k_d_ang = 0.01

maxspeed = 0.1
minspeed = 0.05

maxang = 0.25
minang = -0.25

def move_pose(tar_pose, prev_parts=previous_parts, ostate=old_state):
    global previous_parts, old_state
    old_state = ostate
    previous_parts = prev_parts
    rospy.wait_for_message('/odom', Odometry)	
    rospy.wait_for_message('scan',LaserScan)
    get_odom()
    read_laser_scan_data()
    systemLin = lambda x, v : Control(x,v,maxspeed,minspeed,0, True)
    systemAng1 = lambda yaw, v: Control(yaw,v,maxang,minang,0, False)
    systemAng2 = lambda yaw, v: Control(yaw,v,maxang,minang,0, lin=False, final=True)
    pose = tar_pose 
    move_to_point(systemAng1, k_p_ang, k_d_ang, k_i_ang, pose)
    x_old= move_to_point(systemLin, k_p, k_d, k_i, pose)
    yaw_old = move_to_point(systemAng2, k_p_ang,k_d_ang,k_i_ang,pose)

    xyyaw_old = (x_old[0],x_old[1],yaw_old)
    return previous_parts,xyyaw_old
### END CODE HERE ###
'''if __name__ == '__main__':
    rospy.init_node('navigate',anonymous=True)
    rospy.Rate(10)
    rospy.wait_for_message('/odom', Odometry)	
    get_odom()
    systemLin = lambda x, v : Control(x,v,maxspeed,minspeed,0, True)
    systemAng1 = lambda yaw, v: Control(yaw,v,maxang,minang,0, False)
    systemAng2 = lambda yaw, v: Control(yaw,v,maxang,minang,0, lin=False, final=True)
    point = (0.15,0.15)
    pose = (0.75,0.75,0.0)
    move_to_point(systemAng1, k_p_ang, k_d_ang, k_i_ang, pose)
    move_to_point(systemLin, k_p, k_d, k_i, pose)
    move_to_point(systemAng2, k_p_ang,k_d_ang,k_i_ang,pose)
    '''
   
