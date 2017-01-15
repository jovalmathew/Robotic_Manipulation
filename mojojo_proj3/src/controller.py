#!/usr/bin/env python

###### RUN CATKIN MAKE #######

import rospy
import numpy as np
import math
import time 
import baxter_interface

from std_msgs.msg import *
from mojojo_proj3.srv import * 
from mojojo_proj3.msg import * 

class Controller():
	""" We will have three phases. In phase one, we will initiate the calibration. Next, we will have
	the robot move the blocks around into desired positions. Finally, in phase three, we will either be 
	attacking or defending. Attack mode commences once the ball has been on our side for more than three seconds. 
	Defence mode commences once we finish our throwing motion. 
	The controller class will be checking for all this things to make sure that the robot is in the right state 
	at all times."""
	attack = False
	defend = True
	def __init__(self):

		'''
		# get from parameters
		self.limbSide = rospy.get_param("limbSide")
		self.xiMag= rospy.get_param("xiMag")

		# initialize controller node
		rospy.init_node('controllerMoj', anonymous=True)
		self.rate = rospy.Rate(1)
		self.limb= baxter_interface.limb.Limb(self.limbSide)

		#initialize move_robot service client (of type MoveRobot)
		rospy.wait_for_service('move_robot')
		self.move_robot = rospy.ServiceProxy('move_robot', MoveRobot)  

		#initialize get_end_pose service client (of type GetEndPose)
		rospy.wait_for_service('get_end_pose')
		self.get_end_pose = rospy.ServiceProxy('get_end_pose', GetEndPose)  

		# service initialization for get_end_pose
		rospy.Service('calibration', Calibration, self.handleCalibration)



		# Subscribe to necessary topics:
		# Ball_Coords, Block_Coords
		#rospy.Subscriber("Ball_Coords", WorldCoords, self.ball_callback)

		'''

		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

	def ball_callback(self, data):
		''' This function defines how to handle the location of the ball. Probably just saving it to a variable and 
		returning it. '''
		return

	def handleCalibration(self,data):
		''' Have the robot move to a few spots on the game field. Save these spots as specific loctions
		and send them to the topic Calibration. 
		Points to send over: 
		Ball_Neutral'''

		# Self.get_end_pose returns a array of length 7, giving both position and orientation of Baxters end effector
		Ball_Neutral = self.get_end_pose()

		ballCoord=Ball(Ball_Neutral.endPose.Px, Ball_Neutral.endPose.Py, Ball_Neutral.endPose.Pz)

		return ballCoord

	def nUnitCalc(self,goalXYZ,auxXYZ,startXYZ):
		# plane creation
		planeVec1=np.subtract(auxXYZ,startXYZ)
		planeVec2=np.subtract(goalXYZ,startXYZ)

		nMag=np.linalg.norm(np.cross(planeVec1,planeVec2))
		nVec=np.cross(planeVec1,planeVec2)
		nUnit=np.divide(nVec, nMag)
		if nUnit[2] < 0:
			nUnit = list(np.dot(nUnit, -1))

		return nUnit

		'''
	def recordPoints(self,auxFlag):
		#GOAL POSITION: 
		#wait for 'Enter'
		hold = raw_input('Move to goal position and press enter...')

		# get Goal Point
		goalPose = self.get_end_pose()
		goalConfig = self.limb.joint_angles()

		goaljoints = self.getDictValues(goalConfig)# List of 7 points
		goalXYZ=[goalPose.endPose.Px, goalPose.endPose.Py, goalPose.endPose.Pz]

		#AUX POSITION
		if auxFlag == True:
			# wait for 'Enter'
			hold = raw_input('Move to aux position and press enter...')

		# get Aux Point - this is the "third point" which will define a plane
		auxPose = self.get_end_pose()
		auxXYZ=[auxPose.endPose.Px, auxPose.endPose.Py, auxPose.endPose.Pz]

		#START POSITION
		# wait for 'Enter'
		hold = raw_input('Move to start position and press enter...')

		# get Starting Point
		startPose = self.get_end_pose()
		startXYZ=[startPose.endPose.Px, startPose.endPose.Py, startPose.endPose.Pz]

		return goaljoints, goalXYZ, auxXYZ, startXYZ'''

	def getDictValues(self, jointDict):

		jointList=[]
		jointList.append(jointDict.get(self.limbSide + '_s0'))
		jointList.append(jointDict.get(self.limbSide + '_s1'))
		jointList.append(jointDict.get(self.limbSide + '_e0'))
		jointList.append(jointDict.get(self.limbSide + '_e1'))
		jointList.append(jointDict.get(self.limbSide + '_w0'))
		jointList.append(jointDict.get(self.limbSide + '_w1'))
		jointList.append(jointDict.get(self.limbSide + '_w2'))

		return jointList
	def playBall(self):
		''' This function will take the current state of the world (aka if it's in attack or defense mode)
			and then call methods depending on said state. this is what will be placed in the while loop.''' 
		print("I'm working!!!")

		# Put the following in the main code at the bottom: 

		return
if __name__ == '__main__':
	try:
		controller=Controller()   		
		'''while rospy.not_shutdown:
			if ball_on_side and time_elapsed  > 3:
				controller.defend = False
				controller.attack = True
			else:
				controlloer.defend = True
				controller.attack = False
			controller.playBall()
			'''
			
	except rospy.ROSInterruptException:
		pass

