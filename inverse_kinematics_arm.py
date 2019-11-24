#!/usr/bin/env python
#import rospy
import numpy as np
import matplotlib.pyplot as plt
import scipy.linalg
class ThreeLinkArm:
    
    def __init__(self):
        # Set parameters for the 3-link planar arm
        
        self.theta1 = np.deg2rad(0)
        self.theta2 = np.deg2rad(90)
        self.theta3 = np.deg2rad(90)
        self.l1 = 5
        self.l2 = 3
        self.l3 = 2

    def forwardKinematics(self, theta1, theta2, theta3):
        # Define the homogeneous transformation matrices for the 3-link planar arm
        
        self.theta1 = theta1
        self.theta2 = theta2
        self.theta3 = theta3

        self.t01 = np.matrix([[np.cos(self.theta1), -np.sin(self.theta1), 0, 0],
                        [np.sin(self.theta1), np.cos(self.theta1), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])  
        
        # Implement the transformation matrix from frame {1} to frame {2}
        ### START CODE HERE ###
    
        self.t12 = np.matrix([[np.cos(self.theta2), -np.sin(self.theta2), 0, self.l1],
                        [np.sin(self.theta2), np.cos(self.theta2), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])    
    
        ### END CODE HERE ###


        self.t23 = np.matrix([[np.cos(self.theta3), -np.sin(self.theta3), 0, self.l2],
                        [np.sin(self.theta3), np.cos(self.theta3), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])  
        
        # Implement the transformation matrix from frame {3} to the tip of the end-effector
        ### START CODE HERE ###

        self.t3end = np.matrix([[0, 0, 0, self.l3],
                        [0, 0, 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])      
        
        ### END CODE HERE ###
        
        self.t0end = self.t01*self.t12*self.t23*self.t3end
        
        return self.t0end
    
    def findJointPos(self): 
        # Find the x,y position of each joint and end effector so it can be plotted
        
        # Find the transformation matrices for joint 2 and joint 3
        ### START CODE HERE ###
        trans2 = self.t01*self.t12
        trans3 = trans2*self.t23
        ### END CODE HERE ###
        
        print(trans3)
        
        # Find the x, y coordinates for joints 2 and 3. Put them in a list j2 = [x,y]
        ### START CODE HERE ###
        j2 = [trans2[0,3],trans2[1,3],trans2[2,3]]
        j3 = [trans3[0,3],trans3[1,3],trans3[2,3]]
        ### END CODE HERE ###
        
        endeff = [self.t0end[0,3],self.t0end[1,3],self.t0end[2,3]]
        
        return j2,j3,endeff
        

def plotArm(jnt2pos, jnt3pos, endEffectPos, target=np.array([0,0])):
    # set up figure
    fig = plt.figure(figsize=(4,4))
    ax = fig.add_subplot(111, autoscale_on=False,
                         xlim=(-10, 10), ylim=(-10, 10))
    ax.grid()

    plt.plot(target[0],target[1],'or')
    line, = ax.plot([], [], 'o-', lw=4, mew=5)
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

    line.set_data([], [])
    time_text.set_text('')
    print(endEffectPos)
    x = np.array([0,
                   jnt2pos[0],
                   jnt3pos[0],
                   endEffectPos[0]])
    y = np.array([0,
                   jnt2pos[1],
                   jnt3pos[1],
                   endEffectPos[1]])
    line.set_data((x,y))

    plt.show()
	
arm = ThreeLinkArm()

# Do forward kinematics for a set angle on each joint
T = arm.forwardKinematics(np.deg2rad(45),np.deg2rad(45),np.deg2rad(-45))

# Find the x,y coordinates of joints 2, 3 and end effector so they can be plotted
joint2pos, joint3pos, endEffectorPos = arm.findJointPos()

# Print joint + end effector positions
print("Homogeneous matrix from base to end effector: \n" + str(T))
print("Joint 3 Coordinates: " + str(joint3pos))
print("End effector Coordinates: " + str(endEffectorPos))

# Plot the pose of the arm
plotArm(joint2pos, joint3pos, endEffectorPos)

# compute the geometric Jacobian  
def geomJacobian(jnt2pos, jnt3pos, endEffPos):
    
    ai = np.array([0,0,1])
    col0 = np.array(endEffPos)
    col1 = np.array(endEffPos) - np.array(jnt2pos)
    col2 = np.array(endEffPos) - np.array(jnt3pos)
    J = np.array([np.cross(ai,col0), np.cross(ai,col1), np.cross(ai,col2)]).T 
    print("jac")
    print(J)
    return J
	
# Do forward kinematics for a set angle on each joint
initTheta = np.array([np.deg2rad(45),np.deg2rad(45),np.deg2rad(-45)])
target = np.array([-3,7.5,0])

# compute FK
T = arm.forwardKinematics(initTheta[0], initTheta[1], initTheta[2])
# Find the x,y coordinates of joints 2, 3 and end effector so they can be plotted
joint2pos, joint3pos, endEffectorPos = arm.findJointPos()

# initialize theta
newTheta = initTheta
endEffectorPosInit = endEffectorPos

# define the number of IK iterative steps 
steps = 20

for i in range(steps):
    
    # obtain the Jacobian      
    J = geomJacobian(joint2pos, joint3pos, endEffectorPos)
    
    # compute the dy steps
    newgoal = endEffectorPosInit + (i*(target - endEffectorPosInit))/steps
    deltaStep = newgoal - endEffectorPos
    
    # define the dy
    subtarget = np.array([deltaStep[0], deltaStep[1], 0]) 
    
    # compute dq from dy and pseudo-Jacobian
    ### START CODE HERE
    radTheta = np.linalg.pinv(J) * subtarget
    print(np.linalg.pinv(J))
    print(radTheta)
    radTheta = np.array([sum(radTheta[0]), sum(radTheta[1]), sum(radTheta[2])])
    ### END CODE HERE
    print("rad")
    print(newTheta)
    print(radTheta)
    
    # update the q
    newTheta = newTheta + radTheta
    print(newTheta)

    # ----------- Do forward kinematics to plot the arm ---------------
    # Do forward kinematics for a set angle on each joint
    T = arm.forwardKinematics(newTheta[0],newTheta[1],newTheta[2])

    # Find the x,y coordinates of joints 2, 3 and end effector so they can be plotted
    joint2pos, joint3pos, endEffectorPos = arm.findJointPos()

    print(joint3pos)
    # Plot the pose of the arm
    plotArm(joint2pos, joint3pos, endEffectorPos, target)