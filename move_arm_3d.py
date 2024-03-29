#!/usr/bin/env python
import rospy
from numpy import maximum,minimum
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from time import time, sleep
import math

fullStartAngles = [0, 0, 0, 0, 0]

class RobotArm:
    
    def __init__(self): 
        self.angle1 = np.deg2rad(0)
        self.angle2 = 0.87
        self.angle3 = np.deg2rad(0)
        self.angle4 = np.deg2rad(0)
        #Gripper for angle 5, higher value means more open
        self.angle5 = 1.57
        self.limb1 = 0.08945
        self.limb2diagonal = 0.10595
        self.limb2straight = 0.1
        self.limb2side = 0.035
        self.limb3 = 0.1
        self.limb4 = 0.1076
    def forwardKinematics(self, _angle1, _angle2, _angle3, _angle4):
        # Define the homogeneous transformation matrices for the 3-link planar arm
        
        self.angle1 = _angle1
        self.angle2 = _angle2
        self.angle3 = _angle3
        self.angle4 = _angle4

        self.t01 = np.matrix([[np.cos(self.angle1), 0, np.sin(self.angle1), 0],
                        [0, 1, 0, 0],
                        [-np.sin(self.angle1), 0, np.cos(self.angle1), 0],
                        [0, 0, 0, 1]])  
    
        self.t12 = np.matrix([[np.cos(self.angle2), -np.sin(self.angle2), 0, 0],
                        [np.sin(self.angle2), np.cos(self.angle2), 0, self.limb1],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])    

        self.t23 = np.matrix([[np.cos(self.angle3), -np.sin(self.angle3), 0, self.limb2side],
                        [np.sin(self.angle3), np.cos(self.angle3), 0, self.limb2straight],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]]) 
                        
        self.t34 = np.matrix([[np.cos(self.angle4), -np.sin(self.angle4), 0, self.limb3],
                        [np.sin(self.angle4), np.cos(self.angle4), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])   

        self.t4end = np.matrix([[0, 0, 0, self.limb4],
                        [0, 0, 0, 0],
                        [0, 0, 0, 0],
                        [0, 0, 0, 1]])      
 
        self.t0end = self.t01*self.t12*self.t23*self.t34*self.t4end
        
        return self.t0end
    
    def findJointPos(self): 
        trans2 = self.t01*self.t12
        trans3 = trans2*self.t23
        trans4 = trans3*self.t34

        j2 = [trans2[0,3],trans2[1,3],trans2[2,3]]
        j3 = [trans3[0,3],trans3[1,3],trans3[2,3]]
        j4 = [trans4[0,3],trans4[1,3],trans4[2,3]]
        
        endeff = [self.t0end[0,3],self.t0end[1,3],self.t0end[2,3]]
        
        return j2,j3,j4,endeff

def joint_callback(data):
    global fullStartAngles
    '''print("Msg: {}".format(data.header.seq))
    print("Wheel Positions:\n\tLeft: {0:.2f}rad\n\tRight: {0:.2f}rad\n\n".format(data.position[0],data.position[1]))
    print("Joint Positions:\n\tShoulder1: {0:.2f}rad\n\tShoulder2: {0:.2f}rad\n\tElbow: {0:.2f}rad\n\tWrist: {0:.2f}rad\n\n".format(data.position[2],data.position[3],data.position[4],data.position[5]))
    print("Gripper Position:\n\tGripper: {0:.2f}rad\n".format(data.position[6]))
    print("----------")'''
    fullStartAngles = [data.position[2], data.position[3], data.position[4], data.position[5]]


def read_joint_states():
    rospy.Subscriber("joint_states",JointState,joint_callback)

def clean_joint_states(data):
    #lower_limits = [0, -1.57, -1.57, -1.57, -1.57,   -1]
    #upper_limits = [0,  1.57,  1.57,  1.57,  1.57, 1.57]
    lower_limits = [-1.57, -1.57, -1.57, -1.57]
    upper_limits = [1.57,  1.57,  1.57,  1.57]
    clean_lower = maximum(lower_limits,data)
    clean_upper = minimum(clean_lower,upper_limits)
    return list(clean_upper), np.array_equal(clean_upper, data)

def computeGeomJacobian(jnt2pos, jnt3pos, jnt4pos, endEffPos):
    ai = np.array([0,0,1])
    col0 = np.array(endEffPos)
    col1 = np.array(endEffPos) - np.array(jnt2pos)
    col2 = np.array(endEffPos) - np.array(jnt3pos)
    col3 = np.array(endEffPos) - np.array(jnt4pos)
    J = np.array([np.cross(ai,col0), np.cross(ai,col1), np.cross(ai,col2), np.cross(ai,col3)]).T 

    return J	

def interpolate_angles(curAngles, destAngles, steps, rate):
    rate = rospy.Rate(rate)
    joint_pos = Float64MultiArray()
    #Number of smooth movement iterations
    numIT = steps
    
    prevAngles = curAngles
    
    for i in range(numIT):
        
        curStep1 = prevAngles[0] + i*(destAngles[0] - prevAngles[0])/numIT
        curStep2 = prevAngles[1] + i*(destAngles[1] - prevAngles[1])/numIT
        curStep3 = prevAngles[2] + i*(destAngles[2] - prevAngles[2])/numIT
        curStep4 = prevAngles[3] + i*(destAngles[3] - prevAngles[3])/numIT
        
        newAngles = np.array([curStep1, curStep2, curStep3, curStep4])

        tempAngles, areValidAngles = clean_joint_states(newAngles)
        if areValidAngles == True:
            joint_pos.data = np.concatenate((np.array([0]), np.concatenate((tempAngles, np.array([1.57])))))
            jointpub.publish(joint_pos)
            rate.sleep()
    
def move_arm(goalPos, jointpub, initAngles):

    joint_pos = Float64MultiArray()
#   Joint Position vector should contain 6 elements:
#   [0, shoulder1, shoulder2, elbow, wrist, gripper]
#3rd -1.22 for ground
#0.34 for grip
    initialAngles = initAngles
    
    arm = RobotArm()
    #forward_kinematics = compute_forward_kinematics(joint_states)
    
    # Forward kinematics computation
    target = goalPos

    T = arm.forwardKinematics(initialAngles[0], initialAngles[1], initialAngles[2],
        initialAngles[3])
        
    # x,y coords for all the joints
    joint2pos, joint3pos, joint4pos, endEffectorPos = arm.findJointPos()
    newAngles = initialAngles
    endEffectorPosInit = endEffectorPos

    #Number of IK iterations
    numIT = 20
    
    for i in range(numIT):
        
        # obtain the Jacobian      
        J = computeGeomJacobian(joint2pos, joint3pos, joint4pos, endEffectorPos)

        print("Jointpos")
        print(joint2pos)
        print(joint3pos)
        print(joint4pos)
        print(endEffectorPos)
        # compute the dy steps
        newgoal = endEffectorPosInit + (i*(target - endEffectorPosInit))/numIT
        deltaStep = newgoal - endEffectorPos
        
        # define the dy
        subtarget = np.array([deltaStep[0], deltaStep[1], deltaStep[2]]) 
        
        # compute dq from dy and pseudo-Jacobian
        angleChange = np.dot(np.linalg.pinv(J), subtarget)
        print(angleChange[0])
        angleChange = np.array([angleChange[0], angleChange[1], angleChange[2],
            angleChange[3]])
        
        # update the q
        tempAngles, areValidAngles = clean_joint_states(newAngles + angleChange)
        tempAngles = np.array(tempAngles)
        if areValidAngles == True:
            prevAngles = newAngles
            newAngles = tempAngles
            # Do forward kinematics for a set angle on each joint
            T = arm.forwardKinematics(newAngles[0],newAngles[1],newAngles[2],newAngles[3])

            # Find the x,y coordinates of joints 2, 3 and end effector so they can be plotted
            joint2pos, joint3pos, joint4pos, endEffectorPos = arm.findJointPos()
        
            interpolate_angles(prevAngles, newAngles, 200, 200)
        else:
            print("Hitting rotational limits!")
            continue
    return newAngles

if __name__ == '__main__':
    rospy.init_node('move_arm',anonymous=True)
    rate = rospy.Rate(20)
    jointpub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10) 
    read_joint_states()
    #Wait for 5 sec to get pre-init pos
    time_start = rospy.get_time()
    goal_time = time_start + 5
    while(rospy.get_time() < goal_time):
        rate.sleep()

    initialAngles, placeholder = clean_joint_states([0, 0, 0, 0])
    initialAngles = np.array(initialAngles)
    interpolate_angles(fullStartAngles, initialAngles, 50, 50)
    
    #Wait for 3 sec for arm to get to init pos
    time_start = rospy.get_time()
    goal_time = time_start + 3
    while(rospy.get_time() < goal_time):
        rate.sleep()
        
    #goalLoc = np.array([0.135, 0.28185, 0])
    goalLoc = np.array([0.3426, 0.08945, 0])
    #goalLoc = np.array([0.235, -0.01815, 0])
    #goalLoc = np.array([0.18945, 0.18945, 0])
    finalAngles = move_arm(goalLoc, jointpub, initialAngles)

    '''goalLoc = np.array([0.09, -0.16, 0])
    finalAngles = move_arm(goalLoc, jointpub, finalAngles)'''
    
    #Execute for 5 seconds to let the arm get to destination pos
    time_start = rospy.get_time()
    goal_time = time_start + 5
    while(rospy.get_time() < goal_time):
        rate.sleep()
        
    joint_pos = Float64MultiArray()
    res, pl = clean_joint_states(finalAngles)
    joint_pos.data = np.concatenate((np.array([0]), np.concatenate((res, np.array([0.34])))))
    jointpub.publish(joint_pos)
    
    #Allow for 3 seconds for the grapple to grab the lego, may need smoothing
    time_start = rospy.get_time()
    goal_time = time_start + 3
    while(rospy.get_time() < goal_time):
        rate.sleep()