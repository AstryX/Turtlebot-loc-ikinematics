
#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Nov 11 19:31:13 2019

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

#hard-coded error vals from first MM
error =0.025
error_yaw = 0.05
#Global variables for odom 
yaw=pitch=roll=0.0
x=y=0
prev_x=prev_y=0
#angles for the beam_model
angs = [0,-45,-90,-135,-180,-225,-270,-315] 
#limit of LIDAR in m 
max_range =3.5
#empty array to store measurements 
measurements = np.array(len(angs))
#list of locations for each wall - for readability and debugging
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
            12:"triangle botton",
            13:"triangle right diag",
            14:"triangle left diag"}
# list of walls in format X_start, Y_start, X_end, Y_end.
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
        (1.475,1.55,1.85,2.20),
        (1.10,2.20,1.475,1.55)] 
#store potential walls
#initialise world_co-ords for x,y and yaw
x_init = 4.25-0.42
y_init = 2.15-0.35
#towards window = 0, towards class = 180, towards button = 90, reverse = -90
yaw_init = 90*np.pi/180 
#compute world pose based on odometry 
x_world = x_init-y 
y_world = y_init-x
yaw_world = yaw_init-yaw
#store sensor measurements
readings = []
#convert from local to global co-ords
def tf_global(x=x,y=y,yaw=yaw):
    return x_init-y, y_init-x, yaw_init-yaw
#odometry callback
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
#create odometry subscription
def get_odom():
    rospy.Subscriber('/odom',Odometry,get_rotation)
#laser_scan callback
def laser_scan_callback(data):
    i=0
    for angle in angs:
        measurements[i] = data.ranges[angle]
        i+=1
    
    print(measurements)
#subscribe to LIDAR
def read_laser_scan_data():
    rospy.Subscriber('scan',LaserScan,laser_scan_callback)
#check the angle of an obstacle
def check_angle(pos):
    
    angle = (np.arctan2(y_world-pos[1], x_world-pos[0]))
    return angle
#check distance between 2 points on map
def distance(a,b):
    return np.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)
#check if a point is part of a certain line (obstacle)
def is_between(a,c,b):
    return np.isclose(distance(a,b),(distance(a,c) + distance(c,b)))
# find the wall that Aggron is facing based on above functions
def find_facing_wall():
    wall = []
    for reading in readings:
        for i in range(len(obst)):
            ob_x =(reading*np.cos(yaw_world))+x_world
            ob_y = (reading*np.sin(yaw_world))+y_world
            print ob_x, ob_y
            angle=check_angle((ob_x,ob_y))
            if i>=12:
                print 'angle ='+str(angle)+'('+str(i) 
                print is_between((obst[i][0],obst[i][1]),(ob_x, ob_y),(obst[i][2],obst[i][3]))
            if is_between((obst[i][0],obst[i][1]),(ob_x, ob_y),(obst[i][2],obst[i][3]))and np.isclose(yaw_world,angle):
                wall.append((i,abs(reading)))
    return wall
# calculate the measurement that should be received based on the current estimated state (x,y,yaw)
def get_true_measure():
        global yaw_world 
        if yaw_world == 0 :
            yaw_world = 0.00000001
        for i in range(len(obst)):
            m = ((obst[i][3]-obst[i][1])*(obst[i][0]-x_world)-(obst[i][2]-obst[i][0])*(obst[i][1]-y_world))/(((obst[i][3]-obst[i][1])*np.cos(yaw_world))-((obst[i][2]-obst[i][0])*np.sin(yaw_world)))
            readings.append(m)
        return 

#TODO: TURN THIS INTO A CLASS
class Beam_Model(object):
    def __init__(self, pose, occ_grid, ranges=measurements):
        self.map = occ_grid.data
        self.pose = pose
        self.beams = ranges
        self.zhit = 0.5
        self.zshort = 0.2
        self.zmax = 0.2
        self.zrand = 0.1
        self.zt = 0
        
        self.x_od= x
        self.y_od = y
        self.yaw_od = yaw
        #create weights (zhit, zshort, zmax, zrand) which sum to 1 and prob dists phit(gaussian), pshort(exponential), pmax(uniform), prand (uniform)
        
    def sense(self):
        rospy.wait_for_message('scan',LaserScan)
        read_laser_scan_data()
    return measurements

    def step(self):
        q=1 
        for reading in self.beams:
            #computer the raw measurement 
            self.zt = reading
            # p = zhit*phit + zshort*pshort + zmax*pmax + zrand*prand
            # phit = mean (measurement)  variance - error 0.25
            # pmax = 1 if z==zmax else 0
            #q=q*p
        return q
    
if __name__ == '__main__':
    sys = Beam_Model((x,y,z),maps)
    
    sys.sense
    print sys.step
    x_world = tf_global()[0]
    y_world = tf_global()[1]
    yaw_world = tf_global()[2]
   
