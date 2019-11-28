#!/usr/bin/env python
#By Robertas Dereskevicius 2019/11 University of Edinburgh
#Script that does inverse kinematics for a robot arm with 5 DOF, where first DOF 
#rotates around the yaw and the last DOF is a grappler.
#IK is simplified 2d, where the yaw is passed as an input for the
#first joint before any IK is performed .
import rospy
from numpy import maximum,minimum
import numpy as np
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from time import time, sleep
import math

fullStartAngles = [0, 0, 0, 0, 0]
angleSub = 0

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
        self.limb4 = 0.12915
    def forwardKinematics(self, _angle2, _angle3, _angle4):
        # Define the homogeneous transformation matrices for the 3-link planar arm
        self.angle2 = _angle2
        self.angle3 = _angle3
        self.angle4 = _angle4
    
        self.t12 = np.matrix([[np.cos(self.angle2), -np.sin(self.angle2), 0],
                        [np.sin(self.angle2), np.cos(self.angle2), 0],
                        [0, 0, 1]])    

        self.t23 = np.matrix([[np.cos(self.angle3), -np.sin(self.angle3), self.limb2diagonal],
                        [np.sin(self.angle3), np.cos(self.angle3), 0],
                        [0, 0, 1]]) 
                        
        self.t34 = np.matrix([[np.cos(self.angle4), -np.sin(self.angle4), self.limb3],
                        [np.sin(self.angle4), np.cos(self.angle4), 0],
                        [0, 0, 1]])   

        self.t4end = np.matrix([[0, 0, self.limb4],
                        [0, 0, 0],
                        [0, 0, 1]])      
 
        self.t0end = self.t12*self.t23*self.t34*self.t4end
        
        return self.t0end
    
    def findJointPos(self): 
        trans3 = self.t12*self.t23
        trans4 = trans3*self.t34

        j3 = [trans3[0,2],trans3[1,2]]
        j4 = [trans4[0,2],trans4[1,2]]
        
        endeff = [self.t0end[0,2],self.t0end[1,2]]
        
        return j3,j4,endeff

def joint_callback(data):
    global fullStartAngles
    '''print("Msg: {}".format(data.header.seq))
    print("Wheel Positions:\n\tLeft: {0:.2f}rad\n\tRight: {0:.2f}rad\n\n".format(data.position[0],data.position[1]))
    print("Joint Positions:\n\tShoulder1: {0:.2f}rad\n\tShoulder2: {0:.2f}rad\n\tElbow: {0:.2f}rad\n\tWrist: {0:.2f}rad\n\n".format(data.position[2],data.position[3],data.position[4],data.position[5]))
    print("Gripper Position:\n\tGripper: {0:.2f}rad\n".format(data.position[6]))
    print("----------")'''
    fullStartAngles = [data.position[2], data.position[3], data.position[4], data.position[5]]



def read_joint_states():
    global angleSub
    angleSub = rospy.Subscriber("joint_states",JointState,joint_callback)

def clean_joint_states(data):
    #lower_limits = [0, -1.57, -1.57, -1.57, -1.57,   -1]
    #upper_limits = [0,  1.57,  1.57,  1.57,  1.57, 1.57]
    lower_limits = [-1.56, -1.56, -1.56, -1.56]
    upper_limits = [1.56,  1.56,  1.56,  1.56]
    clean_lower = maximum(lower_limits,data)
    clean_upper = minimum(clean_lower,upper_limits)
    return list(clean_upper), np.array_equal(clean_upper, data)

def computeGeomJacobian(jnt3pos, jnt4pos, endEffPos):
    ai = np.array([0,0,1])
    col0 = np.array(endEffPos + [0])
    col2 = np.array(endEffPos + [0]) - np.array(jnt3pos + [0])
    col3 = np.array(endEffPos + [0]) - np.array(jnt4pos + [0])
    J = np.array([np.cross(ai,col0), np.cross(ai,col2), np.cross(ai,col3)]).T 

    return J	

def interpolate_angles(curAngles, destAngles, steps, rate, gripperRad, jointpub):
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
            joint_pos.data = np.concatenate((np.array([0]), np.concatenate((tempAngles, np.array([gripperRad])))))
            jointpub.publish(joint_pos)
            rate.sleep()
    
def move_arm(goalPos, jointpub, initAngles, gripperRad):

    joint_pos = Float64MultiArray()
#   Joint Position vector should contain 6 elements:
#   [0, shoulder1, shoulder2, elbow, wrist, gripper]
#3rd -1.22 for ground
#0.34 for grip
    fixedYawAngle = [initAngles[0]]
    initialAngles = [initAngles[1] + np.deg2rad(70.71), initAngles[2] - np.deg2rad(70.71), initAngles[3]]
    
    arm = RobotArm()
    #forward_kinematics = compute_forward_kinematics(joint_states)
    
    # Forward kinematics computation
    target = goalPos

    T = arm.forwardKinematics(initialAngles[0], initialAngles[1], initialAngles[2])
        
    # x,y coords for all the joints
    joint3pos, joint4pos, endEffectorPos = arm.findJointPos()
    newAngles = initialAngles
    endEffectorPosInit = endEffectorPos

    #Number of IK iterations
    numIT = 100
    
    for i in range(numIT):
        
        # obtain the Jacobian      
        J = computeGeomJacobian(joint3pos, joint4pos, endEffectorPos)

        print("Jointpos")
        print(joint3pos)
        print(joint4pos)
        print(endEffectorPos)
        # compute the dy steps
        newgoal = endEffectorPosInit + (i*(target - endEffectorPosInit))/numIT
        deltaStep = newgoal - endEffectorPos
        
        # define the dy
        subtarget = np.array([deltaStep[0], deltaStep[1], 0]) 
        
        # compute dq from dy and pseudo-Jacobian
        angleChange = np.dot(np.linalg.pinv(J), subtarget)
        print(angleChange[0])
        angleChange = np.array([angleChange[0], angleChange[1], angleChange[2]])
        
        # update the q 
        changedAngles = newAngles + angleChange
        tempAngles, areValidAngles = clean_joint_states(np.concatenate((fixedYawAngle,
            [changedAngles[0] - np.deg2rad(70.71), changedAngles[1] + np.deg2rad(70.71),
            changedAngles[2]])))
        tempAngles = np.array([tempAngles[1] + np.deg2rad(70.71), tempAngles[2] - np.deg2rad(70.71), tempAngles[3]])
        if areValidAngles == True:
            prevAngles = np.array([newAngles[0] - np.deg2rad(70.71), newAngles[1] + np.deg2rad(70.71), newAngles[2]])
            newAngles = tempAngles
            # Do forward kinematics for a set angle on each joint
            T = arm.forwardKinematics(newAngles[0],newAngles[1],newAngles[2])

            # Find the x,y coordinates of joints 2, 3 and end effector so they can be plotted
            joint3pos, joint4pos, endEffectorPos = arm.findJointPos()
            newAngles = np.array([newAngles[0] - np.deg2rad(70.71), newAngles[1] + np.deg2rad(70.71), newAngles[2]])
        
            interpolate_angles(np.concatenate((fixedYawAngle, prevAngles)), 
                np.concatenate((fixedYawAngle, newAngles)), 10, 100, gripperRad, jointpub)
            newAngles = np.array([newAngles[0] + np.deg2rad(70.71), newAngles[1] - np.deg2rad(70.71), newAngles[2]])
        else:
            print("Hitting rotational limits!")
            continue
    return newAngles

def custom_pause(rate, time):
    time_start = rospy.get_time()
    goal_time = time_start + time
    while(rospy.get_time() < goal_time):
        rate.sleep()

def complete_arm_task(taskType, targetYaw, goal, initAtStart, secondary_goal=[0], secondaryYaw=-99):
    global fullStartAngles
    global angleSub
    #rospy.init_node('move_arm',anonymous=True)
    armTargetYaw = targetYaw
    rate = rospy.Rate(20)
    jointpub = rospy.Publisher('joint_trajectory_point',Float64MultiArray, queue_size =10) 
    read_joint_states()

    gripperRad = 0
    if taskType == "press" or taskType == "push":
        gripperRad = 0.56
    else:
        gripperRad = 1.56

    initialAngles, placeholder = clean_joint_states([0, 0, 0, 0])
    initialAngles = np.array(initialAngles)
    if initAtStart == True:
        #Wait for 3 sec to get pre-init pos
        custom_pause(rate, 3)
        interpolate_angles(fullStartAngles, initialAngles, 50, 50, 1.56, jointpub)

    angleSub.unregister()
        
    #Wait for 1 sec for everything to catch up
    custom_pause(rate, 1)

    postInitAngles, placeholder = clean_joint_states([armTargetYaw, 0, 0, 0])
    postInitAngles = np.array(postInitAngles)
    interpolate_angles(initialAngles, postInitAngles, 50, 50, gripperRad, jointpub)

    #Wait for 1 sec for arm to get to init pos
    custom_pause(rate, 1)
        
    #turtlebot height 15.3 cm ~~
    #pickup lego height 4.3 cm ~~
    #turtlebot + first arm starting point = 24.245 cm

    #goalLoc = np.array([0.1, -0.22945])
    goalLoc = np.array(goal)
    finalAngles = move_arm(goalLoc, jointpub, postInitAngles, gripperRad)
    finalAngles = np.array([finalAngles[0] - np.deg2rad(70.71), finalAngles[1] + np.deg2rad(70.71), finalAngles[2]])
        
    gripper2Rad = gripperRad
    res, pl = clean_joint_states(np.concatenate(([postInitAngles[0]],finalAngles)))

    joint_pos = Float64MultiArray()
    if taskType == "pick":
        #Execute for 1 seconds to let the arm get to destination pos
        custom_pause(rate, 1)
        gripper2Rad = 0.34
        joint_pos.data = np.concatenate((np.array([0]), np.concatenate((res, np.array([gripper2Rad])))))
        jointpub.publish(joint_pos)

    if secondaryYaw != -99:
        #Wait for 1 sec for everything to catch up
        custom_pause(rate, 1)

        postInitAngles2, placeholder = clean_joint_states([secondaryYaw, finalAngles[0], finalAngles[1], finalAngles[2]])
        postInitAngles2 = np.array(postInitAngles2)
        interpolate_angles(np.concatenate(([targetYaw], finalAngles)), postInitAngles2, 50, 50, gripper2Rad, jointpub)    
        res = postInitAngles2

    finalAngles2 = res
        
    if len(secondary_goal) == 2:
        #Allow for 3 seconds for the grapple to grab the lego, may need smoothing
        custom_pause(rate, 3)

        goalLoc2 = np.array(secondary_goal)
        #goalLoc2 = np.array([0.26415, 0.1])
        finalAngles2 = move_arm(goalLoc2, jointpub, res, gripper2Rad) 
        finalAngles2 = np.array([res[0], finalAngles2[0] - np.deg2rad(70.71), finalAngles2[1] + np.deg2rad(70.71), finalAngles2[2]])

    custom_pause(rate, 1)
    initialAngles, placeholder = clean_joint_states([0, 0, 0, 0])
    initialAngles = np.array(initialAngles)
    interpolate_angles(finalAngles2, initialAngles, 50, 50, 1.56, jointpub)


if __name__ == '__main__':
    #complete_arm_task("pick", 0, [0.1, -0.22945], True, [0.26415, 0.1])
    complete_arm_task("press", 0, [0.2, -0.12245], False)
    #complete_arm_task("push", 0, [0.32915, -0.035], False, [0], 1.55)
