#!/usr/bin/env python

import roslib
roslib.load_manifest('mojojo_proj3')
import sys
import rospy
import cv2
import math
import numpy as np
import random

from mojojo_proj3.srv import *
from mojojo_proj3.msg import *
from copy import deepcopy
from std_msgs.msg import UInt16, String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from __future__ import print_function

class RobotVision():

    def __init__(self):

        # defines name of node to rospy
        rospy.init_node("robot_visionMoj", anonymous= True)
        self.rate= rospy.Rate(1) #1 Hz

        rospy.spin()

if __name__ == '__main__':
    try:
        robot_visionMoj= RobotVision()
    except rospy.ROSInterruptException:
        pass