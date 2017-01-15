#!/usr/bin/env python

from __future__ import print_function
import roslib
roslib.load_manifest('mojojo_proj3')
import sys
import rospy
import cv2
import numpy as np

from math import acos, atan2, cos, sin
from mojojo_proj3.msg import *
from mojojo_proj3.srv import *
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError



class image_converter:
	''' This class is instantiated for each image we take, so each image taken by the camera will be entered as an argument
	into this class. Functions in this class take the image and manipulate it to return the location(s) of the object(s) in question.'''
	ROI_Max = 125
	ROI_Min = 50
	Theta_Transf = None
	Scale_X = None
	Scale_Y = None
	O_c = None
	O_r = None
	ball_pixel_coords = None
	Pickup_Xmin = None
	Pickup_Xmax = None
	Pickup_Ymin = None
	Pickup_Ymax = None
	Ball_In_Zone = False

	def __init__(self):

		# defines name of node to rospy
		rospy.init_node("image_converterMoj", anonymous= True)
		self.rate= rospy.Rate(1) #1 Hz

		#initialize get_end_pose service client (of type GetEndPose)
		rospy.wait_for_service('get_end_pose')
		self.get_end_pose = rospy.ServiceProxy('get_end_pose', GetEndPose) 

		self.Ball_Loc = rospy.Publisher("Ball_Loc", Ball,queue_size=10)

		#self.image_pub_raw = rospy.Publisher("moj_raw", Image)
		#self.image_pub_hsv = rospy.Publisher("moj_hsv", Image)

		self.bridge = CvBridge()

		# Subscribe to necessary topics
		self.image_sub = rospy.Subscriber("/camera/rgb/image_color", Image, self.callback)
		#self.imageDepth_sub = rospy.Subscriber("/camera/depth_registered/image", Image, self.depthCallback)
		#self.calib_locs = rospy.Subscriber("Calibration", Calib_Coords, self.calib_callback)
		print('Initializing calibration')
		# Initiate calibration 
		self.initial_calibration()

		#rospy.set_param('MojPhase', 2)
		#rospy.spin()


	def initial_calibration(self):
		#self.depthCallback()

		### GET FIRST PHOTO
		hold = raw_input('Put ball at Point 1, remove obstacles, and press enter...')
		Ball_Calibration1 = self.pink_Mask()

		ball_detected = self.get_Ball_Point(Ball_Calibration1)

		while not ball_detected:
			print("Ball is not detected!")
			hold = raw_input('Ball not detected: try again?')
			Ball_Calibration1 = self.pink_Mask()
			ball_detected = self.get_Ball_Point(Ball_Calibration1)
		Image_Point1 = self.ball_pixel_coords
		#-------------------------------------------------------------



		### GET FIRST ARM LOC
		hold = raw_input('Move arm to Point 1 and press enter...')
		currentEndPose = self.get_end_pose()
		Point1=[currentEndPose.endPose.Px, currentEndPose.endPose.Py]
		#-------------------------------------------------------------



		### GET SECOND PHOTO
		hold = raw_input('Put ball at Center Point, remove obstacles,and press enter...')
		Ball_Calibration2 = self.pink_Mask()
		ball_detected = self.get_Ball_Point(Ball_Calibration2)

		while not ball_detected:
			hold = raw_input('Ball not detected: try again?')
			Ball_Calibration2 = self.pink_Mask()
			ball_detected = self.get_Ball_Point(Ball_Calibration2)

		Image_Point2 = self.ball_pixel_coords
		self.Center_Pix = Image_Point2
		#-------------------------------------------------------------

		### GET SECOND ARM LOC
		### SAVE CENTER BALL POSITION 
		hold = raw_input('Move Arm to center position and press Enter:...')
		currentEndPose = self.get_end_pose()
		Point2=[currentEndPose.endPose.Px, currentEndPose.endPose.Py] 
		self.CenterPose = currentEndPose
		self.Pickup_Xmin = currentEndPose.endPose.Px-.06
		self.Pickup_Xmax = currentEndPose.endPose.Px+.06
		self.Pickup_Ymin = currentEndPose.endPose.Py-.06
		self.Pickup_Ymax = currentEndPose.endPose.Py+.06

		rospy.set_param('centerXYZ', [currentEndPose.endPose.Px, currentEndPose.endPose.Py, currentEndPose.endPose.Pz])
		rospy.set_param('Pickup_Xmin', self.Pickup_Xmin)
		rospy.set_param('Pickup_Xmax', self.Pickup_Xmax)
		rospy.set_param('Pickup_Ymin', self.Pickup_Ymin)
		rospy.set_param('Pickup_Ymax', self.Pickup_Ymax)
		#-------------------------------------------------------------

		### DETERMINE FRAME CONVERSION PARAMETERS
		WorldLocs = [Point1, Point2]
		ImageLocs = [Image_Point1, Image_Point2]
		self.PixeltoWorldParams(WorldLocs, ImageLocs)
		#-------------------------------------------------------------


		### get initial block locations based on end effector location
		hold = raw_input('Ready for PHASE 2: Move Arm over block stack and press Enter:...')
		self.initialEndPose= self.get_end_pose()
		self.initialBlockXYZ = [self.initialEndPose.endPose.Px, self.initialEndPose.endPose.Py, self.initialEndPose.endPose.Pz]
		self.initialBlockQUAT = [self.initialEndPose.endPose.Qx, self.initialEndPose.endPose.Qy, self.initialEndPose.endPose.Qz, self.initialEndPose.endPose.Qw] 
		rospy.set_param('initialBlockXYZ', self.initialBlockXYZ)
		rospy.set_param('initialBlockQUAT', self.initialBlockQUAT)
		#-------------------------------------------------------------





	'''
	def depth_Mask(self, Image):
		
		upperDepthHSV= np.array([341, 100, 100])
		lowerDepthHSV= np.array([300, 0, 0])


		upperDepthHSV=self.convertHSV(upperPinkHSV)
		lowerDepthHSV=self.convertHSV(lowerPinkHSV)

		depthMask= cv2.inRange(self.cv_imageHSV, lowerPinkHSV, upperPinkHSV)
		

		Image = np.array(Image, dtype="uint16")

		depth_array=np.array(Image, dtype=np.float32)
		cv2.imshow("Depth Raw ", depth_array )

		return depth_array
		
		depthMask=cv2.threshold(self.cv_imageDepth,127,255,cv2.THRESH_BINARY)

		cv2.imshow("Depth Raw ", self.cv_imageDepth )
		cv2.imshow("Depth Mask ", depthMask )
		waitKey(1)
	'''

	def blue_Mask(self):
		''' Image is a picture that gets returned with the callback function '''

		# define range of blue color in HSV
		#VALUES FOR BLUE BLOCKS
		#HSV Values for Blue, probably not good: 221,63,95

		upperBlueHSV= np.array([250, 100, 100])
		lowerBlueHSV= np.array([200, 40, 70])
		
		upperBluekHSV=self.convertHSV(upperBlueHSV)
		lowerBlueHSV=self.convertHSV(lowerBlueHSV)

		blueMask= cv2.inRange(self.cv_imageHSV, lowerBlueHSV, upperBlueHSV) 
		#BlockMaskedImage = cv2.imshow("BlockMaskedImage", blueMask)


		Block_Filtered = self.morphological_filtering(blueMask)
		cv2.imshow("Block_Filtered", Block_Filtered)
		return Block_Filtered

	def pink_Mask(self):
		# gimp value ranges, H:[0 360], S:[0 100], V:[0 100]
		#gimpPinkHSV= np.array([330, 76, 78])
		#VALUES FOR PINK BALL
		upperPinkHSV= np.array([341, 100, 100])
		lowerPinkHSV= np.array([300, 40, 75])

		upperPinkHSV= np.array([334, 100, 100])
		lowerPinkHSV= np.array([300, 40, 75])

		upperPinkHSV=self.convertHSV(upperPinkHSV)
		lowerPinkHSV=self.convertHSV(lowerPinkHSV)



		pinkMask= cv2.inRange(self.cv_imageHSV, lowerPinkHSV, upperPinkHSV)
		#BallMaskedImage= cv2.imshow("BallMaskedImage", pinkMask)

		# Do some morphological filtering 
		Ball_Filtered = self.morphological_filtering(pinkMask)
		cv2.imshow("Ball_Final_Filtered", Ball_Filtered)
		return Ball_Filtered
		#return pinkMask

	def PixeltoWorldParams(self,WorldLocs,PixelLocs):
		# This function takes two lists, WorldLocs and PixelLocs, and creates parameters that can be used 
		# to convert pixel coordinates to World Coordinates. These parameters won't change during an instance
		# this class, so these can be saved as attributes.
		# Rename inputs for clarity
		# Perhaps we need to convert them into arrays
		PointA_World = np.array(WorldLocs[0])
		PointA_Image = np.array(PixelLocs[0])

		PointB_World = np.array(WorldLocs[1])
		PointB_Image = np.array(PixelLocs[1])

		# Find a scaling factor between the size of each pixel vs the size of world coordinates
		AB_World = PointB_World - PointA_World
		AB_Image = PointB_Image - PointA_Image
		self.Scale_X = AB_World[0]/AB_Image[0]
		self.Scale_Y = AB_World[1]/AB_Image[1]

		return

	def convertPixelToWorldCoords(self,PixelCoords):
		# This function requires an input of type list with two entries [u,v], and converts it to a list with two entries [x,y]
		# Probably has array vs. list syntax issues!
		PixelCoords = np.array(PixelCoords) # Does this work?
		Center_Pix_Array = np.array(self.Center_Pix)

		Image_Diff = Center_Pix_Array - PixelCoords   # Take difference between center pixels and current pixels.
		# Now, we don't need o_r, o_c. We can use the center location as our reference.
		x = Image_Diff[1]*self.Scale_X + self.CenterPose.endPose.Px
		y = Image_Diff[0]*self.Scale_Y + self.CenterPose.endPose.Py


		WorldCoords = [x, y]

		return WorldCoords 
		
	def get_Block_Points(self, Block_Filtered):
		self.createBlockDetector()
		block_keypoints = self.blockDetector.detect(Block_Filtered)
		Block_img_keypoints = cv2.drawKeypoints(Block_Filtered, block_keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		
		cv2.imshow("Block_keypoints ", Block_img_keypoints )

		if len(block_keypoints) > 0:
			# Multiple blocks should be detected, so all of their locations should be returned. Find all of these coordinates, block_pixel_coords,
			# and save them as a list of lists. 
			block_pixel_coords = []
			
			for i in range(len(block_keypoints)-1):
				block_pixel_coords.append(block_keypoints[i].pt)

		return block_pixel_coords

	def get_Ball_Point(self, Ball_Filtered):

		self.createBallDetector()       


		# Detect balls & blocks.
		ball_keypoints = self.ballDetector.detect(Ball_Filtered)
		Ball_img_keypoints = cv2.drawKeypoints(Ball_Filtered, ball_keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
		cv2.imshow("Ball_keypoints ", Ball_img_keypoints )


		if len(ball_keypoints) > 0:
			# Almost always, there is only one blob detected by the processor. Therefore, just return the coordinates of the first element in
			# ball_keypoints as a tuple
			self.ball_pixel_coords = ball_keypoints[0].pt   
			#print('%d Balls Detected' % len(ball_keypoints))
			return True
		else:
			#print('Ball Not Detected')
			return False

	def callback(self,data):
		
		#self.image_pub_raw.publish(data)

		try:
			cv_imageBGR = self.bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			print(e)

		# convert from BGR8 to HSV 
		cv_imageHSV_temp= cv2.cvtColor(cv_imageBGR, cv2.COLOR_BGR2HSV)
		image_size = cv_imageHSV_temp.shape

		# We'll cut out the parts of the image that contain Baxter
		self.cv_imageHSV = cv_imageHSV_temp[self.ROI_Min:image_size[0]-self.ROI_Max , self.ROI_Min:image_size[1]-self.ROI_Min]


		# display images
		#cv2.imshow("cv_imageHSV", self.cv_imageHSV)
		cv2.waitKey(1)  

	'''
	def depthCallback(self,data):
		
		# Use cv_bridge() to convert the ROS image to OpenCV format
		try:
			# The depth image is a single-channel float32 image
			depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
		except CvBridgeError as e:
			print(e)

		# Convert the depth image to a Numpy array since most cv2 functions
		# require Numpy arrays.
		depth_array = np.array(depth_image, dtype=np.float32)
				
		# Normalize the depth image to fall between 0 (black) and 1 (white)
		depth_arrayNorm = cv2.normalize(depth_array, depth_array, 0, 1, cv2.NORM_MINMAX)
		
		# Display the result
		cv2.imshow("Depth Image", depth_array)

		cv2.waitKey(1) 
	'''


	def morphological_filtering(self, Image):
		

		kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(1,1))
		opening = cv2.morphologyEx(Image, cv2.MORPH_OPEN, kernel)
		#cv2.imshow("opening", opening)

		kernel = cv2.getStructuringElement(cv2.MORPH_RECT,(2,2))
		closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
		#cv2.imshow("closing", closing)


		return closing

	def convertHSV(self, gimpHSV):
		# gimpHSV is of type Array and so is the cvHSV type 

		cvHSV=gimpHSV

		cvHSV[0]=gimpHSV[0]*180/360
		cvHSV[1]=gimpHSV[1]*255/100
		cvHSV[2]=gimpHSV[2]*255/100

		return cvHSV

	def createBallDetector(self):
		# sets parameters for ball detection, and creates ball detector
		# Setup SimpleBlobDetector parameters.
		ballParams = cv2.SimpleBlobDetector_Params()
 
		# Change thresholds
		ballParams.minThreshold = 5;
		ballParams.maxThreshold = 255;
		 
		# Filter by Color
		ballParams.filterByColor = False
		ballParams.blobColor = 255

		# Filter by Area.
		ballParams.filterByArea = True
		ballParams.minArea = 50
		ballParams.maxArea = 140         

		# Filter by Circularity
		# perfect circle has circularity = 1, square has circularity = 0.785
		ballParams.filterByCircularity = True   
		ballParams.minCircularity = 0.5
		 
		# Filter by Convexity
		ballParams.filterByConvexity = True
		ballParams.minConvexity = 0.5
		 
		# Filter by Inertia
		ballParams.filterByInertia = False
		ballParams.minInertiaRatio = 0.6
		
		# Create a detector with the parameters
		ver = (cv2.__version__).split('.')
		if int(ver[0]) < 3 :
			self.ballDetector = cv2.SimpleBlobDetector(ballParams)
		else :
			self.ballDetector = cv2.SimpleBlobDetector_create(ballParams)
			
	def createBlockDetector(self):
			# sets parameters for block detection, and creates block detector
			# Setup SimpleBlobDetector parameters.
			blockParams = cv2.SimpleBlobDetector_Params()
	 
			# Change thresholds
			blockParams.filterByColor = False
			blockParams.minThreshold = 10;
			blockParams.maxThreshold = 200;
			 
			# Filter by Area.
			blockParams.filterByArea = True
			blockParams.minArea = 100
			blockParams.maxArea = 400        
	
			# Filter by Circularity
			# perfect circle has circularity = 1, square has circularity = 0.785
			blockParams.filterByCircularity = False
			blockParams.maxCircularity = 0.785
			 
			# Filter by Convexity
			blockParams.filterByConvexity = True
			blockParams.minConvexity = 0.7
			 
			# Filter by Inertia
			blockParams.filterByInertia = True
			blockParams.minInertiaRatio = 0.5
			
			# Create a detector with the parameters
			ver = (cv2.__version__).split('.')
			if int(ver[0]) < 3 :
				self.blockDetector = cv2.SimpleBlobDetector(blockParams)
			else :
				self.blockDetector = cv2.SimpleBlobDetector_create(blockParams) 

	def trackBall(self):
		'''This method will take the current Image, perform masking to find the ball, get it's coordinates, and check if it's 
		in the pick up zone. '''

		# If we don't find the ball in this method, we can return the old ball coordinates. 
		Old_Ball_Coords = self.convertPixelToWorldCoords(self.ball_pixel_coords)

		#print('Tracking ball')
		find_ball = self.pink_Mask()

		ball_found = self.get_Ball_Point(find_ball) # Returns True or False, depending if the ball is found 

		if ball_found:
			# Convert to baxter's coordinates, and publish to a topic that keeps track of past points. 
			Ball_Coords = self.convertPixelToWorldCoords(self.ball_pixel_coords)
			#print(Ball_Coords)

			self.Ball_Loc.publish(Ball_Coords)
			
			if Ball_Coords[0] > self.Pickup_Xmin and Ball_Coords[0] < self.Pickup_Xmax and Ball_Coords[1] > self.Pickup_Ymin and Ball_Coords[1] < self.Pickup_Ymax:
				self.Ball_In_Zone = True
				print('$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$  IN ZONE')
			
			
			else:
				# Reset the timer
				self.Ball_In_Zone = False
				'''
				self.startTime = 0
				self.currentTime = 0 
				'''

			return Ball_Coords
		else:
			return Old_Ball_Coords


def main(args):

	ic = image_converter()

	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == '__main__':
	try:
		
		### START PHASE 0
		#this loop will keep us from moving forward until phase 0 is complete
		while rospy.get_param('MojPhase')==0:
			x = 1

		### START PHASE 1
		# initialize class, do phase 1 stuff
		image_converterMoj= image_converter()

		### START PHASE 2
		while rospy.get_param('MojPhase')<3:
			x=1

		### START PHASE 3
		while not rospy.is_shutdown():
			# Get ball position
			ball_coords = image_converterMoj.trackBall()
			'''print('ball_coords')
			print(ball_coords)

			print('World Pose')
			print(image_converterMoj.get_end_pose())
			print([image_converterMoj.Theta_Transf, image_converterMoj.Scale_X, image_converterMoj.Scale_Y, image_converterMoj.O_c, image_converterMoj.O_r])
			rospy.sleep(1)'''
			###################################
			# This chunk of code will determine if the ball in is the pickup zone, ready to be picked up.
			if image_converterMoj.Ball_In_Zone == True:
				startTime = rospy.get_time()
				currentTime = rospy.get_time()

				while currentTime - startTime < 2.5:
					ball_coords = image_converterMoj.trackBall()
					if image_converterMoj.Ball_In_Zone == False:
						break
					currentTime = rospy.get_time()
				if currentTime - startTime > 2.5:
					rospy.set_param('DefenceOrAttack_Mode', 1)
					x=1

			else:
				startTime = 0
				currentTime = 0
			###################################




	except rospy.ROSInterruptException:
		pass 