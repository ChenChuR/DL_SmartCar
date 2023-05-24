#! /usr/bin/env python3.6


# import ctypes
# import os
# import shutil
# import random
import sys
# import threading
#import time
import cv2
import numpy as np
# import pycuda.autoinit
# import pycuda.driver as cuda
# import tensorrt as trt
# import argparse
# import os

import rospy
from std_msgs.msg import String
from serial_pkg.msg import speed
import cv_bridge
#from cv_bridge import CvBridge
from sensor_msgs.msg import Image



#sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages')
#sys.path.append('/usr/local/lib/python3.6/dist-packages')


def CallBack(imgmsg):
    #try:
        rospy.loginfo("Image_sub Start")    
        img = cv_bridge.CvBridge().imgmsg_to_cv2(imgmsg, "bgr8")
        print(1)
    



if __name__ == "__main__":
    # Node setting
    rospy.init_node("test")      
    # load labels
    #categories = ["sidewalk", "rate-limiting-on","left-turn", "ramp", "rate-limiting-off"]


    rate = rospy.Rate(20)
    rospy.Subscriber("camera/road_image", Image, CallBack)
    rospy.loginfo("Node_sub Initialization Finished")
    rospy.spin()
