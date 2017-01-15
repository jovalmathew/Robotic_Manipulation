#!/usr/bin/env python

import rospy
import baxter_interface
import time 
import math
import numpy as np
import random
import cv2
import rospkg

from game_server.srv import *
from game_server.msg import *
from mojojo_proj3.srv import *
from mojojo_proj3.msg import *
from baxter_pykdl import baxter_kinematics
from std_msgs.msg import UInt16, String, Header
from baxter_interface import CHECK_VERSION
from copy import deepcopy
from baxter_core_msgs.srv import ( 
	SolvePositionIK,
	SolvePositionIKRequest,
)
from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Point,
	Quaternion,
)
from cv_bridge import CvBridge, CvBridgeError


class RobotInterface():
	block_Xmax = None
	block_Xmin = None

	Defense_Xmax = None
	Defense_Xmin = None
	Defense_Xhome = None
	Defense_Y = None
	ball_loc = None
	Arm = 1

	def __init__(self):

		

		# defines name of node to rospy
		rospy.init_node("robot_interfaceMoj", anonymous= True)
		self.rate= rospy.Rate(1) #1 Hz

		# subscribe to ball_topic, which gives the location of the ball.
		rospy.Subscriber("Ball_Loc", Ball, self.ball_callback)

		#subscribe to game state topic
		rospy.Subscriber("/game_server/game_state", GameState, self.game_state_callback)

		# service initialization for move_robot
		rospy.Service('move_robot', MoveRobot, self.handleMoveRobot)

		# service initialization for move_robot
		rospy.Service('move_blocks', MoveBlocks, self.handleMoveBlocks)

		# service initialization for get_end_pose
		rospy.Service('get_end_pose', GetEndPose, self.handleGetEndPose)

		#initialize get_end_pose service client (of type GetEndPose)
		rospy.wait_for_service('get_end_pose')
		self.get_end_pose = rospy.ServiceProxy('get_end_pose', GetEndPose) 

		#initialize move_blocks service client (of type GetEndPose)
		rospy.wait_for_service('move_blocks')
		self.move_blocks = rospy.ServiceProxy('move_blocks', MoveBlocks) 



		#initialization info to the game server
		rospack = rospkg.RosPack()
		impath = rospack.get_path('mojojo_proj3') + '/Mojojo.png'
		img = cv2.imread(impath)
		imMsg = CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
		self.game_server_proxy = rospy.ServiceProxy('/game_server/init', Init)
		rospy.wait_for_service('/game_server/init')
		response = self.game_server_proxy('Mojojo', imMsg)
		self.limbSide = response.arm
		rospy.set_param("limbSide", self.limbSide)
		
		### COMMENT THIS OUT LATER
		#self.limbSide='left' 
		#rospy.set_param("limbSide", 'left')



		# get from parameters
		#self.limbSide = rospy.get_param("limbSide")
		self.xiMag= rospy.get_param("xiMag")
		self.numBlocks= rospy.get_param("num_blocks")
		#self.numBlocks = 5
		print "The number of blocks is:"
		print self.numBlocks
		self.blockHeight= 0.04445

		if rospy.get_param("limbSide") == 'right':
			self.Arm = -1


		print("response.arm")
		print(response.arm)


		# enable robot
		print("Getting robot state... ")
		self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
		self._init_state = self._rs.state().enabled

		print("Enabling robot... ")
		self._rs.enable()
		
		# creates an instance of baxter_interface called interface
		# http://sdk.rethinkrobotics.com/wiki/Baxter_Interface
		self.limb= baxter_interface.limb.Limb(self.limbSide)

		# creates an instance of baxter_kinematics called kin
		# http://sdk.rethinkrobotics.com/wiki/Baxter_PyKDL
		self.kin= baxter_kinematics(self.limbSide)

		# sets the class timestep
		self.dT=0.002 #500 Hz

		# Create list that will track ball's trajectory
		self.trajectory = []

		#rospy.spin()
	def game_state_callback(self, data):
		""" Read the phase of the game (0,1,2,3) from the game server, and sets it as a parameter so that our code can use it"""
		rospy.set_param("MojPhase", data.current_phase)
		print("Current Phase")
		print(rospy.get_param('MojPhase'))


	def modeScattered(self):

		Block_Range = range(self.numBlocks,0,-1)

		for i in Block_Range:
			self.move_blocks("moveToBlock", i)
			self.move_blocks("closeGripper", i)
			self.move_blocks("moveOverBlock", -i)
			self.move_blocks("openGripper",i)

		#move_robot("moveToBlock",1,'right')
		
	def ball_callback(self, data):
		''' When a new ball location comes in, this function is called. 
		Here, we will determine the ball trajectory and return the ball coordinates
		'''
		
		self.ball_loc = list(data.pos)

	def determineScatterPositions(self):
		self.scatterLocation = []
		self.scatterLocation.append(Block(self.initialEndPose.Px,self.initialEndPose.Py,self.initialEndPose.Pz-self.numBlocks*self.blockHeight))
		

		self.scatterLocation.append(Block(self.center[0], self.center[1]-self.Arm*0.2, self.initialEndPose.Pz-self.numBlocks*self.blockHeight))

		
		i = 1

		while i <= np.ceil(self.numBlocks/2):

			self.scatterLocation.append(Block(self.center[0]+0.10*i, self.center[1]-self.Arm*0.15, self.initialEndPose.Pz-self.numBlocks*self.blockHeight))
			self.scatterLocation.append(Block(self.center[0]-0.10*i, self.center[1]-self.Arm*0.15, self.initialEndPose.Pz-self.numBlocks*self.blockHeight))

			i = i + 1

		print "self.scatterLocation"
		print self.scatterLocation

		'''
		if self.limbSide =="right":
			for i in range (0, self.numBlocks):
				self.scatterLocation.append(Block(self.initialEndPose.Px-.3+0.05*i,self.initialEndPose.Py+.2,self.initialEndPose.Pz-self.numBlocks*self.blockHeight))
		
			print "Successfully Determined Scatter Positions"
			print "self.scatterLocation"
			print self.scatterLocation
		'''

	def handleMoveBlocks(self,req):

		Action=req.action
		Target=req.target

		

		rospy.loginfo("Entered handleMoveBlocks")



		if Action == "openGripper":
			rospy.loginfo("Opening Gripper")
			baxter_interface.Gripper(self.limbSide).open()
			rospy.sleep(1)

		elif Action == "closeGripper":
			rospy.loginfo("Closing Gripper")
			baxter_interface.Gripper(self.limbSide).close()
			rospy.sleep(1)

		elif Action == "moveToBlock":

			rospy.loginfo("Moving To Block")
			TargetPos = self.blockLocations[Target]
			'''
			print "TargetPos"
			print TargetPos
			'''
			# Move to the Target, hitting the home positions on the way
			self.move_home(Target)
			

			# Recalibrate gripper
			baxter_interface.Gripper(self.limbSide).calibrate()
		   
			# Move to Target position
			desired_limb_joints=self.randomWalk_IK(TargetPos.x, TargetPos.y, TargetPos.z)
			rospy.loginfo("MOVING TO BLOCK %i", Target)
			baxter_interface.Limb(self.limbSide).move_to_joint_positions(desired_limb_joints)
		

		elif Action == "moveOverBlock": 
			rospy.loginfo("MOVING OVER BLOCK %i", Target)
			self.move_home(Target)
			

			# Move over Target
			if Target > 0:
				rospy.loginfo("Target SHOULD BE > 0")
				TargetPos = currentState.block[Target]
				ik_test(limb, TargetPos.x, TargetPos.y, TargetPos.z+blockHeight)
				baxter_interface.Limb(limb).move_to_joint_positions(desired_limb_joints)
				mode = rospy.get_param('modeParam')



				if configuration == "stacked_descending" or mode == "stacked_ascending":
					currentState.block[Target+1] = endpointState[limbInt].pose.position
				elif configuration == "stacked_ascending" or mode == "stacked_descending":
					currentState.block[Target-1] = endpointState[limbInt].pose.position

			
			elif Target ==0:
				rospy.loginfo("BUT Target == %i", Target)
				TargetPos = blockZeroCoords[0]
				ik_test(limb, TargetPos.x, TargetPos.y, TargetPos.z+blockHeight)
				baxter_interface.Limb(limb).move_to_joint_positions(desired_limb_joints)
				mode = rospy.get_param('modeParam')


				if configuration == "stacked_descending" or mode == "stacked_ascending":
					currentState.block[1] = endpointState[limbInt].pose.position
				elif configuration =="stacked_ascending" or mode == "stacked_descending":
					currentState.block[num_blocks] = endpointState[limbInt].pose.position
					rospy.loginfo("WE HAVE ENTERED TARGET 0 COORDINATE REASSIGNMENT")
					rospy.loginfo(endpointState[limbInt].pose.position)
			
			elif Target < 0:
				rospy.loginfo("TARGET < 0 ????")



				TargetPos = self.scatterLocation[abs(Target)]

				desired_limb_joints=self.randomWalk_IK(TargetPos.x, TargetPos.y, TargetPos.z+self.blockHeight)
				baxter_interface.Limb(self.limbSide).move_to_joint_positions(desired_limb_joints)

				#update block coordinates

				currentEndPose = self.get_end_pose()

				self.blockLocations[abs(Target)].x = currentEndPose.endPose.Px
				self.blockLocations[abs(Target)].y = currentEndPose.endPose.Py
				self.blockLocations[abs(Target)].z = currentEndPose.endPose.Pz

		return True
	
	def move_home(self, Target):
		
		zDelta = 0.06+self.blockLocations[abs(Target)].z
		rospy.loginfo('Moving Home')
		
		currentEndPose = self.get_end_pose()
		desired_limb_joints=self.randomWalk_IK(currentEndPose.endPose.Px, currentEndPose.endPose.Py, zDelta) 
		baxter_interface.Limb(self.limbSide).move_to_joint_positions(desired_limb_joints)

		if Target <= 0:
			currentEndPose = self.get_end_pose()
			desired_limb_joints=self.randomWalk_IK(self.scatterLocation[abs(Target)].x, self.scatterLocation[abs(Target)].y, currentEndPose.endPose.Pz) 
		
		else:
			currentEndPose = self.get_end_pose()
			desired_limb_joints=self.randomWalk_IK(self.blockLocations[abs(Target)].x, self.blockLocations[abs(Target)].y, currentEndPose.endPose.Pz) 

		baxter_interface.Limb(self.limbSide).move_to_joint_positions(desired_limb_joints)
			
	def IK_move(self,xNew, yNew, zNew):


		# initialize IK service and request
		ns = "ExternalTools/" + self.limbSide + "/PositionKinematicsNode/IKService"

		iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
		ikreq = SolvePositionIKRequest()


		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	  
		poses = {
			
			'left': PoseStamped(
				header=hdr,
				pose=Pose(
					position=Point(
						x=xNew,
						y=yNew,
						z=zNew,
					),
					orientation=Quaternion(
						x=self.initialEndPose.Qx,
						y=self.initialEndPose.Qy,
						z=self.initialEndPose.Qz,
						w=self.initialEndPose.Qw,
					),
				),
			),
			
			
			'right': PoseStamped(
				header=hdr,
				pose=Pose(
					position=Point(
						x=xNew,
						y=yNew,
						z=zNew,
					),
					orientation=Quaternion(
						x=self.initialEndPose.Qx,
						y=self.initialEndPose.Qy,
						z=self.initialEndPose.Qz,
						w=self.initialEndPose.Qw,
					),
				),
			),
		}
		

		ikreq.pose_stamp.append(poses[self.limbSide])

		try:
			rospy.wait_for_service(ns, 5.0)
			resp = iksvc(ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return 1

		if (resp.isValid[0]):
			print("SUCCESS - Valid Joint Solution Found:")

			# Format solution into Limb API-compatible dictionary
			desired_limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			
		else:
			print("INVALID POSE - No Valid Joint Solution Found.")
			desired_limb_joints = [] 
	 
		return desired_limb_joints
	
	def randomWalk_IK(self, xNew, yNew, zNew):
		''' This function calls IK, and determines a plan of action when IK fails'''
		
		Pickup_Xmin= rospy.get_param('Pickup_Xmin')
		Pickup_Xmax= rospy.get_param('Pickup_Xmax')
		Pickup_Ymin= rospy.get_param('Pickup_Ymin')
		Pickup_Ymax= rospy.get_param('Pickup_Ymax')

		desired_limb_joints = self.IK_move(xNew, yNew, zNew)


		while desired_limb_joints == []: 

			# first loop through x values
			while desired_limb_joints == []:

				# set new x
				if (xNew+.1 <= Pickup_Xmax) and (xNew +.1 >= Pickup_Xmin):
					xNew = xNew + .1
				else:
					xNew = Pickup_Xmin

				desired_limb_joints = self.IK_move(xNew, yNew, zNew)	
				
				if desired_limb_joints == []:				

					if (yNew+.1 <= Pickup_Ymax) and (yNew +.1 >= Pickup_Ymin):
						yNew = yNew + .1
					else:
						yNew = Pickup_Ymin

			desired_limb_joints = self.IK_move(xNew, yNew, zNew)


			if desired_limb_joints == [] and self.limbSide == 'right':
				print('going into hard coded position')
				baxter_interface.Limb(self.limbSide).move_to_joint_positions({'right_s0': -0.11351457817382814, 'right_s1':-0.4835874428283692 , 'right_w0':0.7366942725402833 , 'right_w1': 0.974077799194336, 'right_w2': 0.3861796629089356 , 'right_e0': 0.4118738410766602, 'right_e1':1.7924565485961916 })
			elif desired_limb_joints == [] and self.limbSide == 'left':
				baxter_interface.Limb(self.limbSide).move_to_joint_positions({'left_s0':-0.2952913013305664 , 'left_s1': -0.6427379493896485, 'left_w0':-1.223349676940918 , 'left_w1':1.1470341328308107 , 'left_w2': -0.2297136227233887 , 'left_e0':-0.04141748122558594 , 'left_e1':1.810480822833252})
		return desired_limb_joints

	def initialBlockXYZ(self):

		
		self.blockLocations= []
		self.blockLocations.append(Block(self.initialEndPose.Px, self.initialEndPose.Py, self.initialEndPose.Pz-self.numBlocks*self.blockHeight))

		xBlockCoords= [self.initialEndPose.Px]*(self.numBlocks+1)
		yBlockCoords= [self.initialEndPose.Py]*(self.numBlocks+1)
		zBlockCoords= [self.initialEndPose.Pz]*(self.numBlocks+1)

		for i in range(self.numBlocks,0,-1):

			zBlockCoords[i]=self.initialEndPose.Pz-(i-1)*self.blockHeight
			self.blockLocations.append(Block(xBlockCoords[i], yBlockCoords[i], zBlockCoords[i]))

	def assignJointVel(self, qdot):
		# assigns joint velocity components

		# initializing joint velocity dictionary (values do not mean anything)
		jointVel= self.limb.joint_velocities()

		# builds desired dictionary of qdot components
		jointVel[self.limbSide + '_s0'] = qdot[0]
		jointVel[self.limbSide + '_s1'] = qdot[1]
		jointVel[self.limbSide + '_e0'] = qdot[2]
		jointVel[self.limbSide + '_e1'] = qdot[3]
		jointVel[self.limbSide + '_w0'] = qdot[4]
		jointVel[self.limbSide + '_w1'] = qdot[5]
		jointVel[self.limbSide + '_w2'] = qdot[6]

		return jointVel

	def handleMoveRobot(self, req):
		rospy.loginfo('Entered handleMoveRobot')

		goalXYZ = req.goal
		nUnit = req.nUnit
		motionType = req.motionType

		if motionType == 'straight':
			rospy.loginfo("motionType=straight") 
			self.pointToPoint(goalXYZ)

		else:
			rospy.loginfo('PROGRAMMED MSG: motionType is not spelled right')
		return True

	def pointToPoint(self,goalXYZ):
		# This function will take two points, startXYZ, and goalXYZ, and 

		# get current end effector XYZ position
		currentEndPose = self.get_end_pose()
		currentXYZ=[currentEndPose.endPose.Px, currentEndPose.endPose.Py, currentEndPose.endPose.Pz]
		
		# Defines start point (Used for the proportional controller. doesn't change, unlike CurrentXYZ)
		startXYZ= currentXYZ

		# This variable will allow us to stop the robot arm after a certain distance. It represents the 
		# magnitude of the distance traveled. 
		distance_traveled = 0

		rMag,rUnit=self.getMagUnit(goalXYZ)

		# solve for linear velocity vector, magnitude xi
		desired_vVec = np.multiply(rUnit,self.xiMag)
		desired_wVec = np.array([0.0, 0.0, 0.0])

		# concatenating linear and angular velocity vectors
		desired_xiVec= np.matrix(list(desired_vVec) + list(desired_wVec))

		# get time trajectory starts to execute
		startTime= rospy.get_time()
		rospy.loginfo("Starting trajectory at time: %f", startTime)

		# scale for error
		kp = 0.7 #decent

		# tolerace from goalXYZ
		tol= 0.005
		tolVec=np.array([tol,tol,tol])

		# condition compares each component of each vector, returns true or false
		while distance_traveled < rMag:

			# calculate time elapsed since startTime
			currentTime=rospy.get_time()
			deltaT = currentTime-startTime

			# calculate ideal distance traveled in deltaT (returns three component vector)
			deltaXYZ = np.multiply(desired_vVec, deltaT)

			# calculate desired XYZ after time step deltaT
			desiredXYZ = np.add(startXYZ, deltaXYZ)

			# calculate position error
			errorXYZ = np.subtract(desiredXYZ, currentXYZ)  

			# establish second linear velocity vector with error term
			scaledError = np.multiply(errorXYZ,kp)
			vVec_effective = np.add(desired_vVec, scaledError)

			# concatenating linear and angular velocity vectors
			xiVec_effective= np.matrix(list(vVec_effective)+list(desired_wVec))

			# given our desired body velocity, calculate b
			b=self.calculateB_2(xiVec_effective)
			#b=np.zeros((7,1))

			# scales the arbitrary b-vector
			#b=np.dot(b,15)

			# creates matrix type
			bMatrix=np.matrix(b)
			bMatrix=bMatrix.reshape((7,1))

			# solves for joint velocities
			qdot = self.solve_qdot(xiVec_effective,bMatrix)
			jointVel=self.assignJointVel(qdot)

			#self.limb.set_command_timeout(0.01)
			self.limb.set_joint_velocities(jointVel)

			# get current end effector XYZ position
			currentEndPose = self.get_end_pose()
			currentXYZ=[currentEndPose.endPose.Px, currentEndPose.endPose.Py, currentEndPose.endPose.Pz]

			# Update the distance traveled
			distance_traveled = np.linalg.norm(np.subtract(currentXYZ,startXYZ)) # Take the two vectors, subract them, take their magnitude

			if distance_traveled >=rMag:
				self.limb.exit_control_mode()
				break   

			rospy.sleep(self.dT)
		
			if distance_traveled >=rMag:
				self.limb.exit_control_mode()
				break
			
	def solve_qdot(self,desired_xiVec,b):
		
		# solve current jacobian pseudo inverse 
		jacPseudoInverse= self.kin.jacobian_pseudo_inverse()

		#############################
		jac=self.kin.jacobian()

		J=np.array(jac)
		JPlus=np.array(jacPseudoInverse)

		I=np.mat(np.eye(7))

		IminusJPlusJ=np.subtract(I,np.dot(JPlus,J))

		#############################

		# solve for qdot
		qdot = np.dot(jacPseudoInverse,desired_xiVec.transpose()) + np.dot(IminusJPlusJ,b)

		return qdot

	def calculateB_2(self,xiVec_effective):
		#get current joint angles, manipulability
		qDict=self.limb.joint_angles()
		qCurr=self.getDictValues(qDict)
		mCurr=self.find_manipulability(qCurr)
		
		#define deltaAngle (radians)
		dAngle=0.02
		
		#initialize b 
		b=np.zeros((7,1))


		b[0]=0.5 * self.Arm		

			
		return list(b)     

	def calculateB(self,xiVec_effective):
		
		#get current joint angles, manipulability
		qDict=self.limb.joint_angles()
		qCurr=self.getDictValues(qDict)
		mCurr=self.find_manipulability(qCurr)
		
		#define deltaAngle (radians)
		dAngle=0.02
		
		#initialize b 
		b=np.zeros((7,1))
		
		#qCurr is the current configuration
		for i in range (0,7):
			qPotential = deepcopy(qCurr)    
			qPotential[i] = qCurr[i]+dAngle
			mPotential=self.find_manipulability(qPotential)
			b[i]=np.divide(np.subtract(mPotential,mCurr),dAngle)
			
		return list(b)      

	def find_manipulability(self,q):
		# Input: a list of joint angles; Output: a float which describes manipulability

		#initialize qDict to be in the correct "dictionary" format
		qDict=self.limb.joint_angles()

		#pack up q list into qDict dictionary
		qDict[self.limbSide + '_s0'] = q[0]
		qDict[self.limbSide + '_s1'] = q[1]
		qDict[self.limbSide + '_e0'] = q[2]
		qDict[self.limbSide + '_e1'] = q[3]
		qDict[self.limbSide + '_w0'] = q[4]
		qDict[self.limbSide + '_w1'] = q[5]
		qDict[self.limbSide + '_w2'] = q[6]

		#find jacobian for configuration described by qDict
		J= self.kin.jacobian(qDict)
		Jt= self.kin.jacobian_transpose(qDict)
		
		#calculate manipulability 
		manip=np.sqrt(np.linalg.det(J*Jt))
		
		return manip

	def getMagUnit(self, goalXYZ):

		# get current end effector XYZ position
		currentEndPose = self.get_end_pose()
		currentXYZ=[currentEndPose.endPose.Px, currentEndPose.endPose.Py, currentEndPose.endPose.Pz]

		# solve for position unit vector 
		rVec = np.subtract(goalXYZ, currentXYZ)
		rMag = np.linalg.norm(rVec)
		rUnit = np.divide(rVec,rMag)

		return rMag, rUnit

	def handleGetEndPose(self,req):
		#Command returns a dictionary
		#pose = {'position': (x,y,z), 'orientation': (x,y,z,w)}
		
		endDict=self.limb.endpoint_pose()
		endList= list(endDict['position']) + list(endDict['orientation'])
		
		#convert dictionary to msg type EndPose
		currentEndPose=EndPose(endList[0],endList[1],endList[2],endList[3],endList[4],endList[5],endList[6])

		return currentEndPose

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

	def Ball_Path(self): 
		''' This method first stores some amount of ball positions, which can be found by subscribing to the 
		topic ball_loc. It will then take this list and compute a probable ball trajectory. Using this trajectory, 
		we can position our arm position to best defend. '''
		# Determine length of trajectory list
		trajectory_length = 50
		N = 5 # This number represents the indexing used in our slope calculations 
		while len(self.trajectory) < trajectory_length and type(self.ball_loc) == list:
			self.trajectory.append(self.ball_loc)
			#rospy.sleep(.1)

		self.trajectory.append(self.ball_loc)
		self.trajectory.pop(0) # Delete the first item from the list

		if type(self.ball_loc) == list:
			# Determine Slope of trajectory

			# Average the first ten points in the trajectory to obtain a more accurate point of reference. 
			num_to_average = 10.

			x_interSum = 0
			y_interSum = 0

			for a in range(0,int(num_to_average)):
				x_interSum = x_interSum + self.trajectory[a][0]
				y_interSum = y_interSum + self.trajectory[a][1]

			interpX = x_interSum/num_to_average
			interpY = y_interSum/num_to_average

			'''  In this next section, we calculate slopes between every N points (to reduce risk of Divide by Zero errors. 
				Once all the slopes are calculated, we compute an average slope, and calculate the projected blocking point 
				by the equation (y2-y1)/(x2-x1) = slope'''

			if self.trajectory[trajectory_length -1][0] - self.trajectory[0][0] < .002 : # Account for Divide by Zero Errors
				return self.Defense_Xhome

			else:
				#slope = (self.trajectory[9][1] - self.trajectory[0][1])/(self.trajectory[9][0] - self.trajectory[0][0])
				slopeRunningTotal = 0.0

				for i in range(0,trajectory_length-N,N):
					if self.trajectory[i+N][0] - self.trajectory[i][0] > 0.002:
						slopeRunningTotal = (self.trajectory[i+N][1] - self.trajectory[i][1])/(self.trajectory[i+N][0] - self.trajectory[i][0]) + slopeRunningTotal
					else:
						slopeRunningTotal = slopeRunningTotal+100
				slopeAverage = slopeRunningTotal/(trajectory_length/N)

			Defending_X = interpX + (self.Defense_Y-interpY)/slopeAverage

			# Account for bounces
			if Defending_X > self.Defense_Xmax:
				Defending_X = self.Defense_Xmax - (Defending_X - self.Defense_Xmax)
				return Defending_X
			elif Defending_X <self.Defense_Xmin:
				Defending_X = self.Defense_Xmin - (Defending_X - self.Defense_Xmin)
				return Defending_X
			else:
				return Defending_X
		else:
			print('No ball location')
			return self.Defense_Xhome

	def Shot_1(self):
		''' This method defines the basic shot that our arm will take. '''

		#move just above center point
		Center_Point = rospy.get_param('centerXYZ')

		print('Center_Point')
		print(Center_Point)

		print('IN SHOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTTT 1')


		desired_limb_joints=self.IK_move(Center_Point[0], Center_Point[1], Center_Point[2]+.1)
		baxter_interface.Limb(self.limbSide).move_to_joint_positions(desired_limb_joints)

		# recalibrate gripper
		baxter_interface.Gripper(self.limbSide).calibrate()

		#move around ball
		desired_limb_joints=self.IK_move(Center_Point[0], Center_Point[1], Center_Point[2])
		baxter_interface.Limb(self.limbSide).move_to_joint_positions(desired_limb_joints)

		#close gripper
		self.move_blocks("closeGripper", 1)

		#raise the ball
		desired_limb_joints=self.IK_move(Center_Point[0], Center_Point[1], Center_Point[2]+.1)
		baxter_interface.Limb(self.limbSide).move_to_joint_positions(desired_limb_joints)

		#begin shot motion
		#currentEndPose=self.get_end_pose()
		#self.pointToPoint([Center_Point[0],Center_Point[1]+.1,currentEndPose.endPose.Pz])

		'''
		while True:
			angles = baxter_interface.Limb(self.limbSide).joint_angles()
			right_w1_value=angles[self.limbSide+'_w1']
			print (angles)
			rospy.sleep(1)
		'''

		if self.limbSide == 'right':

			#print("RIGHT SIDE FOR SHOT!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
			jointVel= self.limb.joint_velocities()
			#jointVel[self.limbSide + '_s0'] = qdot[0]
			#jointVel[self.limbSide + '_s1'] = qdot[1]
			#jointVel[self.limbSide + '_e0'] = qdot[2]
			#jointVel[self.limbSide + '_e1'] = qdot[3]
			jointVel[self.limbSide + '_w1'] = 10
			#jointVel[self.limbSide + '_w1'] = qdot[5]
			#jointVel[self.limbSide + '_w2'] = qdot[6]

			#baxter_interface.Limb(self.limbSide).move_to_joint_positions({'right_s0': 0.9752282847839356, 'right_s1': -0.40727189871826175, 'right_w0': 1.2804904612243653, 'right_w1': 1.3525875581726075, 'right_w2': 0.026461168560791018, 'right_e0': -1.3146215337158205, 'right_e1': 1.9113400595214844})
			#baxter_interface.Limb(self.limbSide).move_to_joint_positions({'right_s0': 0.0981747703125, 'right_s1': -1.0845244157958984, 'right_w0': 0.6484903773376465, 'right_w1': 0.7282573782165528, 'right_w2': 0.02300971179199219, 'right_e0': -0.6285486271179199, 'right_e1': 2.2890828281066895})
			#baxter_interface.Limb(self.limbSide).move_to_joint_positions({'right_s0': 0.0981747703125, 'right_s1': -1.0845244157958984, 'right_w0': 0.6484903773376465, 'right_w1': 0.3282573782165528, 'right_w2': 0.02300971179199219, 'right_e0': -0.6285486271179199, 'right_e1': 2.2890828281066895})
			baxter_interface.Limb(self.limbSide).move_to_joint_positions({self.limbSide + '_s0': 0.5767767755859375, self.limbSide + '_s1': -0.8038059319335938, self.limbSide + '_w0': 1.1677428734436035, self.limbSide + '_w1': 0.7217379598754883, self.limbSide + '_w2': -0.023776702185058594, self.limbSide + '_e0': -0.7669903930664063, self.limbSide + '_e1': 2.024087647302246})

			right_w1_value= 99

			#right_w1': 1.842310924145508
			right_w1_cutoff=0.342310924145508
			right_w1_cutoff=0.042310924145508
			right_w1_cutoff=1.2

			baxter_interface.Limb(self.limbSide).set_command_timeout(1)

			while right_w1_value >right_w1_cutoff:
				#update right_w1_value
				angles = baxter_interface.Limb(self.limbSide).joint_angles()
				right_w1_value=angles[self.limbSide + '_w1']

				self.limb.set_joint_velocities(jointVel)

			baxter_interface.Gripper(self.limbSide).open(block=False)
			rospy.sleep(1)


		if self.limbSide == 'left':


			jointVel= self.limb.joint_velocities()
			#jointVel[self.limbSide + '_s0'] = qdot[0]
			#jointVel[self.limbSide + '_s1'] = qdot[1]
			#jointVel[self.limbSide + '_e0'] = qdot[2]
			#jointVel[self.limbSide + '_e1'] = qdot[3]
			jointVel[self.limbSide + '_w0'] = -7
			#jointVel[self.limbSide + '_w1'] = qdot[5]
			#jointVel[self.limbSide + '_w2'] = qdot[6]


			baxter_interface.Limb(self.limbSide).move_to_joint_positions({'left_w0': 2.6434323897033694, 'left_w1': 1.557373993121338, 'left_w2': -1.61029633024292, 'left_e0': -1.3004322114440918, 'left_e1': 1.4285196070861816, 'left_s0': 0.15109710743408203, 'left_s1': -0.7236554358581544})
			baxter_interface.Limb(self.limbSide).move_to_joint_positions({'left_w0': 2.6434323897033694, 'left_w1': 1.557373993121338, 'left_w2': -1.61029633024292, 'left_e0': -1.3004322114440918, 'left_e1': 1.3285196070861816, 'left_s0': 0.15109710743408203, 'left_s1': -0.7236554358581544})
					
			left_w0_value=99															 
			left_w0_cutoff =0.485121423614502
			left_w0_cutoff =1

			baxter_interface.Limb(self.limbSide).set_command_timeout(1)

			while left_w0_value > left_w0_cutoff:
				#update right_w1_value
				angles = baxter_interface.Limb(self.limbSide).joint_angles()
				left_w0_value=angles[self.limbSide + '_w0']

				self.limb.set_joint_velocities(jointVel)

			baxter_interface.Gripper(self.limbSide).open(block=False)
			rospy.sleep(1)


			#self.limb.exit_control_mode()
				
				#rospy.sleep(0.002)
			'''
				if right_w1_value >right_w1_cutoff*0.9:
					baxter_interface.Gripper(self.limbSide).open(block=False, timeout=0.001)
					rospy.sleep(1)
					#self.limb.exit_control_mode()
					#break

				rospy.sleep(0.001)
				
				if right_w1_value >right_w1_cutoff*0.5:
					baxter_interface.Gripper(self.limbSide).open(block=False, timeout=0.001)
					rospy.sleep(1)
					#self.limb.exit_control_mode()
					#break
			'''
			#baxter_interface.Gripper(self.limbSide).open(block=False, timeout=0.001)
				

			#open gripper
			#self.move_blocks("openGripper", 1)
			#self.limb.exit_control_mode()


		#put arm back in good position for defense mode
		desired_limb_joints=self.IK_move(Center_Point[0], Center_Point[1], Center_Point[2]+.1)
		baxter_interface.Limb(self.limbSide).move_to_joint_positions(desired_limb_joints)


if __name__ == '__main__':
	try:

		robot_interface= RobotInterface()


		### START PHASE 0
		#this loop will keep us from moving forward until phase 0 is complete
		#while rospy.get_param('MojPhase')==0:
		#	x = 1

		### START PHASE 1
		#this loop will keep us from moving forward until phase 1 is complete
		while rospy.get_param('MojPhase')<=1:
			#just loop here until image_converter is finished initializing. 
			x = 1
			#print rospy.get_param('MojPhase')

		

		### START PHASE 2
		robot_interface.center = rospy.get_param('centerXYZ') 
		initialBlockXYZ=rospy.get_param('initialBlockXYZ')
		initialBlockQUAT=rospy.get_param('initialBlockQUAT')
		robot_interface.initialEndPose=EndPose(initialBlockXYZ[0],initialBlockXYZ[1],initialBlockXYZ[2],initialBlockQUAT[0],initialBlockQUAT[1],initialBlockQUAT[2], initialBlockQUAT[3])
		
		#this gets the initial block coordinates
		robot_interface.initialBlockXYZ()

		#this gets the desired block scattered locations
		robot_interface.determineScatterPositions()
		
		#this performs the scatter operation series
		robot_interface.modeScattered()

		#rospy.set_param('MojPhase',3)

		#this loop will keep us from moving forward until phase 2 is complete
		while rospy.get_param('MojPhase')<3:
			x=1


		### START PHASE 3
		
		robot_interface.block_Xmax = robot_interface.center[0] + .33
		robot_interface.block_Xmin = robot_interface.center[0] - .33

		robot_interface.Defense_Xhome = robot_interface.center[0]
		robot_interface.Defense_Xmin = robot_interface.center[0] - .14
		robot_interface.Defense_Xmax = robot_interface.center[0] + .14
		robot_interface.Defense_Y = robot_interface.center[1] + .13*robot_interface.Arm

		#start reposition timer for defense
		startTimeRepose = rospy.get_time()

		#this loop will keep us in Phase 3 until the end of time
		while not rospy.is_shutdown():
			DefAttMode = rospy.get_param('DefenceOrAttack_Mode')
			
			print('center spot')
			print(robot_interface.center[0], robot_interface.center[1])

			if DefAttMode == 1:
				# Go into Attack Mode and do something
				print('#######################################')
				print('#######################################')
				print('#######################################')
				print('#######################################')
				print('#######################################')
				print('in attack')
				print('#######################################')
				print('#######################################')
				print('#######################################')
				print('#######################################')


				
				robot_interface.Shot_1()

				rospy.set_param('DefenceOrAttack_Mode', 0)

				#start reposition timer for defense
				startTimeRepose = rospy.get_time()

			elif DefAttMode == 0:
				# go into defense mode and do things 
				#print('in defense')
				print('in defend')
				print('in defend')
				print('in defend')

				Defend_x = robot_interface.Ball_Path()

				
				currentTimeRepose = rospy.get_time()

				if currentTimeRepose-startTimeRepose>15:
					''' Because Baxter"s arm has a tendency to drag into the ground (due to drifting), we reposition baxters arm 
					periodically (every 10 seconds)'''
					#we reposition
					if robot_interface.limbSide == 'right':
						baxter_interface.Limb(robot_interface.limbSide).move_to_joint_positions({'right_s0':-0.7121505799621582 , 'right_s1':0.21590779564819337 , 'right_w0':-1.7947575197753907 , 'right_w1':1.296980754675293 , 'right_w2': 1.5826846760925295 , 'right_e0':1.3502865869934082 , 'right_e1':1.4810584490112306 })
					elif robot_interface.limbSide == 'left':
						baxter_interface.Limb(robot_interface.limbSide).move_to_joint_positions({'left_w0':1.6892963407287598 , 'left_w1':1.3464516350280762 , 'left_w2':-1.4335050446411133 , 'left_e0': -1.4396409677856445 , 'left_e1':1.2455923983398438 , 'left_s0': 0.5261554096435547, 'left_s1':0.22664566115112306})

					
					#reset time
					startTimeRepose = rospy.get_time()
					currentTimeRepose = rospy.get_time()
				
				if Defend_x == None:
					Defend_x = robot_interface.Defense_Xhome

				robot_interface.pointToPoint([Defend_x,robot_interface.Defense_Y, robot_interface.center[2]+0.02])
			else:
				# Do some third thing to determine what's wrong 
				print('i, baxter, do not know what is going on.')


	except rospy.ROSInterruptException:
		pass


