#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Nov 12 16:28:03 2019

@author: tommy

"""
import numpy as np
import math 
import rospy
import random 
from time import time

wall_loc = {0:"top long wall",
            1:"top right diag corner",
            2:"right side wall - long",
            3:"bottom wall indent - short",
            4:"right side wall indent - short",
            5:"bottom wall long",
            6:"bottom left diag corner",
            7:"left side wall",
            8:"rect left side",
            9:"rect bottom",
            10:"rect top",
            11:"rect right side",
            12:"triangle bottom",
            13:"triangle left diag",
            14:"triangle right diag"}
obst = [(0.,0.,3.95,0.),
        (3.95,0.,4.25,0.3),
        (4.25,0.3,4.25,2.15),
        (3.20,2.15,4.25,2.15),
        (3.20,2.15,3.20,3.2),
        (0.6,3.20,3.20,3.20),
        (0.0,2.50,0.6,3.20),
        (0.0,0.0,0.0,2.50),
        (1.05,0.75,1.05,0.91),
        (1.05,0.91,3.20,0.91),
        (1.05,0.75,3.20,0.75),
        (3.20,0.75,3.20,0.91),
        (1.10,2.20,1.85,2.20),
        (1.10,2.20,1.475,1.55),
        (1.475,1.55,1.85,2.20)] # list of walls in format X_start, Y_start, X_end, Y_end.
obst= np.array(obst)


x_world = 4.25-0.42
y_world = 2.15-0.35
error_yaw = 0.05
error = 0.025
yaw_world = 90*np.pi/180 #towards window = 0, towards class = 180, towards button = 90, reverse = -90
readings = []
wall = []
N = 10
prev_parts = np.zeros((N,3))
weights = np.ones(N)
x_init = 4.25-0.42
y_init = 2.15-0.35

beams = [1.8,
         2.06,
         0.49,
         0.35,
         0.49,
         0.56,
         1.25,
         0.8]
beam_int = 45*np.pi/180
#towards window = 0, towards class = 180, towards button = 90, reverse = -90
yaw_init = 90*np.pi/180 

def check_angle(pos):
    
    angle = (np.arctan2(y_world-pos[1], x_world-pos[0]))
    return angle

def distance(a,b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

def is_between(a,c,b):
    #print distance(a,b), distance(a,c)+distance(c,b)
    return np.isclose(distance(a,b),(distance(a,c) + distance(c,b)))

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
def sample_norm(b_2 ):
    b= np.sqrt(b_2)
    rand = np.random.uniform(-b,b,12)
    return 0.5 * sum(rand) 
    
def sample_tri(b_2):
    b= np.sqrt(b_2)
    rand = np.random.uniform(-b,b,2)
    return (np.sqrt(6)/2) * sum(rand) 


def perp( a ) :
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return 
def perp( a ) :
    b = np.full_like(a,a[:,::-1])
    b[:,0] = -a[:,1]
   
    #b[:,1] = a[:,0]
    return b

# line segment a given by endpoints a1, a2
# line segment b given by endpoints b1, b2
# return 
def seg_intersect(a1,a2, b1,b2) :
    da = a2-a1
   
    db = b2-b1
   
    dp = a1-b1
 
    dap = perp(da)
    #print dap.T
    denom =np.dot( dap, db.T)
 
    num = np.dot( dap, dp.T )
   
   
    #print num/denom
    f  = (num/denom)*np.identity(len(num/denom))
    t = np.dot(f,db)
    t = t+b1
    #print '*******'+'\n'+str(np.round(t,2))+'\n'+'*******'

    
    #print b1
    test=db
    int1 = (num/denom)[0,1]*db
    int2 = (num/denom)[1,0]*db
    #print num/denom
    
    #print 'int1 is = '+str(int1)
    #print 'int2 is = '+str(int2)   
    #print int1 + b1[0,0]
    #print int2 +b1 [0,1]
    return t


def get_true_measure2(x,y,yaw):
        global yaw_world 
        global reading
        closest_wall=0
        
        reading = 3.5
        if yaw_world == 0 :
            yaw_world = 0.00000001
        i =0
        pos_reads = np.full_like((obst),(x,y,(np.cos(yaw)*3.5+x),(np.sin(yaw)*3.5)+y))
        '''print (pos_reads)'''
        obst1= obst[:,0:2]
        obst2= obst[:,2:]
        p1 = pos_reads[:,0:2]
        p2 = pos_reads[:,2:]
        
        
            
        inter = seg_intersect(obst1,obst2,p1,p2)
        #print 'intersection = ' + str(inter)
        '''
            print p2
            print inter
            print wall_loc[i]
            print yaw*180/np.pi
            print np.arctan2(inter[1]-y,inter[0]-x)*180/np.pi
            print is_between(obst1,inter,obst2)'''
        #print inter.any(0)
        for ind,inters in enumerate(inter):
            if is_between(obst1[ind],inters,obst2[ind]):
            #m=((obst[i][3]-obst[i][1])*(obst[i][0]-x)-(obst[i][2]-obst[i][0])*(obst[i][1]-y))/(((obst[i][3]-obst[i][1])*np.cos(yaw))-((obst[i][2]-obst[i][0])*np.sin(yaw)))
                
                curr= distance(inter[ind],p1[ind])
                '''
                print curr
                print np.arctan2(y-inter[1],x-inter[0])*180/np.pi
                print '-------'
                print np.isclose(np.arctan2(inter[1]-y,inter[0]-x),yaw, rtol = 0.25)
        print'@ ------@
                '''
                if yaw <-180*np.pi/180:
                    yaw +=180
                if yaw >180:
                    yaw -=180
                if curr<reading and (np.isclose(np.arctan2(inters[1]-y,inters[0]-x),yaw, rtol = 0.25)or np.isclose(np.arctan2(inters[1]-y,inters[0]-x),yaw,rtol=0.25)):
                    
                    reading = curr
                    closest_wall = ind
                    #print 'working: ' + ' reading = '+str(reading) + ' wall = ' + str(closest_wall) 
        if reading >3.5:
                closest_wall= 'nan'
        #print reading 
        
        return reading,closest_wall

def likelihood2(x,m,std):
    if x>0 and x<3.5:
        val=(1/np.sqrt(2*np.pi*std))*(np.exp(-0.5*((x-m)**2/std)))
       
        return val
    else:
        return 0 
def maxmeas(beam):
    if beam ==3.5:
        return 1
    else:
        return 0
def randmeas(beam):
    if beam >=0 and beam <3.5:
        return 1/3.5
    else:
        return 0 
    
#*****************************#
beams2 = beams
#beams2[1:]=reversed(beams[1:])
#******************************#
def sample_measurement(xs,ys,yaws,lidar2):
    p=1
    pos_w = tf((xs,ys,yaws))
    #print (pos_w)
    #print ('------')
    xs = pos_w[0]
    ys = pos_w[1]
    yaws =pos_w[2]
    
    #reading,closest_wall = get_true_measure2(xs,ys,yaws)
    #reading,closest_wall = get_true_measure2(xs,ys,yaws)
   # print reading
    global beams
  
    #print beams2
    for ind,beam in enumerate(beams2):
        reading,closest_wall = get_true_measure2(xs,ys,yaws+(ind*beam_int))
        #print ('true measure = ' +str(reading) + ' for beam ' + str(((ind*beam_int)*180/np.pi)))
        #print ('beam measure = ' + str(beam))
    #closest_wall = min(wall,key=lambda t: t[1])[0]
        q=(1.0*likelihood2(beam, abs(reading), 0.015))+(0.75*maxmeas(beam))+ (0.5*randmeas(beam))
        p*=q
        # print 'beam prob = ' + str(q) + ' for beam ' + str(((ind*beam_int)*180/np.pi))
        
        #print p
    #yaw+= (2*np.pi)/len(beams)
    #if closest_wall != 'nan':
        #print 'Aggron is facing: '+str(wall_loc[closest_wall])
    #else: 
        #print 'oor'
    #print 'total prob for all measurements = ' + str(p)
    return p

def sample_motion(control=0, prev_xt=(0,0,0)):
    #prev_x prev_y and prev_theta passed in from last known state X_t-1 e.g. particle 
    prev_x = prev_xt[0]
    prev_y = prev_xt[1]
    prev_theta = prev_xt[2]
    #x1,x2,theta represent xt_1 parameters from the odometer - taken from control
    #x2,y2,theta_des represent desired state for odometery data. - taken from control 
    theta = control[0][2]
    x1= control[0][0]
    y1 = control[0][1]
    x2 = control[1][0]
    y2 = control[1][1]
    theta_des = control[1][2]
    # error paramaters a1-a3 
    a1 = 0.025
    a2 = 0.025
    a3 = 0.001
    a4 = 0.001
    angle_1 = np.arctan2(y2-y1,x2-x1)-theta
    angle_2 = theta_des - theta - angle_1
    t_dist = distance((x1,y1),(x2,y2))
    
    
    #print 'angles'+str(angle_1)+' '+str(angle_2)+' '+str(t_dist)
    
    sd1 = np.power(a1*angle_1,2)+np.power(a2*t_dist,2)
    sd2 = np.power(a4*angle_1,2)+np.power(a3*t_dist,2)+np.power(a4*angle_2,2)
    sd3 = np.power(a1*angle_2,2)+np.power(a2*t_dist,2)
    
    
    p_rot1,p_trans,p_rot2 = (angle_1 - sample_norm(sd1), t_dist-(sample_tri(sd2)),
                             angle_2 - sample_norm(sd3))    
 
   # print p_rot1,p_trans,p_rot2
    x_est = prev_x+(p_trans*np.cos(prev_theta + p_rot1))
    y_est = prev_y + (p_trans*np.sin(prev_theta+p_rot1))
    theta_est =  prev_theta + p_rot1 + p_rot2
    #print x_est,y_est,theta_est
    return x_est, y_est, theta_est
#return a list of particles estimated poses (x,y,yaw)
def move_particles(control=((0,0,0),(0,0,0)),previous_parts=prev_parts,beams2=beams): 
    st=time()
    particles =temp_particles= []
    global beams 
    beams = beams2
    global weights
    weights = np.ones(N)
    i=0
    x1 =control[0][0]
    y1 = control[0][1]
    yaw1 = control[0][2]
    x2 = control[1][0]
    y2 = control[1][1]
    yaw2 = control[1][2]
    pose = (x1,y1,yaw1)
    des_pose = (x2,y2,yaw2)
    control = (pose,des_pose)
    weights_n=np.zeros(N)
    while i<N:
        state_i = sample_motion(control, previous_parts[i])
        temp_particles.append((state_i[0],state_i[1],state_i[2]))
        weights[i] *= sample_measurement(state_i[0],state_i[1],state_i[2],beams2)
        i+=1
    temp_particles = np.array(particles)
    #print temp_particles
    #print weights
    #print weights.sum()
    weights_n = weights/weights.sum()
    #print "weights_n  = " + str(weights_n)
    
    j=0
    bins = np.cumsum(weights_n)
    while j<N:
        r = random.uniform(0,1)
        index = np.digitize(r,bins)
        particles[j] = temp_particles[np.digitize(r, bins)]
        # print('index of drawn part is ' + str(index))
        j+=1
     
    
    global prev_parts
    prev_parts= particles 
    curr=time()
    #print 'time in pf: '+str(curr-st)
    return particles

'''if __name__ == "__main__":

    #get_true_measure()
    #find_facing_wall()
    #closest_wall = min(wall,key=lambda t: t[1])[0]
    #print "x \t y \t z"
    #print str(x_world) + '\t' + str(y_world) + '\t' + str(yaw_world*180/np.pi)
    #print 'Aggron is facing: '+str(wall_loc[closest_wall])
    #print len(obst)
    #print wall
    #print readings[closest_wall], readings[13]
    #print likelihood(lidar, abs(readings[closest_wall]), 0.02)
    start  = time.time()
    control=((0,0,0),(0,0.15,0*np.pi/180),1.8)
    beams[2] += 0.15
    beams[4] +=0.15
    parts = move_particles(control)
    
    #print parts
    state_est = np.mean(parts, axis=0)
    end = time.time()
    print(end - start)
    print state_est
    true_est,closest = get_true_measure2(tf(state_est)[0],tf(state_est)[1],tf(state_est)[2])
    print wall_loc[closest]
    #
    #print state_est[2]*180/np.pi
    print tf(state_est)[0],tf(state_est)[1],tf(state_est)[2]*180/np.pi
    step2 = time.time()
    control=((state_est[0],state_est[1],state_est[2]),(0,0.3,0*np.pi/180),1.8)
    beams[2] += 0.15
    beams[4] +=0.15
    beams[1] +=0.2
    beams[-1]-= 0.2
    beams[-2] -= 0.15
    
    parts= move_particles(control,prev_parts)
    #print parts
    state_est = np.mean(parts,axis=0)
    end2  = time.time()
    print (end2-step2)
    print state_est
    true_est,closest = get_true_measure2(tf(state_est)[0],tf(state_est)[1],tf(state_est)[2])
    print wall_loc[closest]
    #print state_est[2]*180/np.pi
    #print tf(state_est)[0],tf(state_est)[1],tf(state_est)[2]*180/np.pi'''
    
    
