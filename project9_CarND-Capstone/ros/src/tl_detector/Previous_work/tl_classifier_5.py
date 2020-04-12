from styx_msgs.msg import TrafficLight
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import scipy.misc

i_width = 600
i_height = 800

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass
        
   
    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        light = TrafficLight.UNKNOWN
        # HSV allows count colort within hue range
        hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
        hsv_img = scipy.misc.imresize(hsv_img,(i_height, i_width))
        
        #print "The image size is {}".format(hsv_img.shape)
        
        RED1 = np.array([0,100,100], np.uint8)
        RED2 = np.array([15,255,255], np.uint8)
        RED3 = np.array([155,100,100], np.uint8)
        RED4 = np.array([180,255,255], np.uint8)
        
        YELLOW1 = np.array([38.0/360*255, 100,100], np.uint8)
        YELLOW2 = np.array([65.0/360*255, 255,255], np.uint8)
        
        GREEN1 = np.array([0.9*255, 100,100], np.uint8)
        GREEN2 = np.array([0.2*255, 255,255], np.uint8)
        
        red_mask1 = cv2.inRange(hsv_img, RED1, RED2)
        red_mask2 = cv2.inRange(hsv_img, RED3, RED4)
        yellow_mask = cv2.inRange(hsv_img, YELLOW1, YELLOW2)
        green_mask = cv2.inRange(hsv_img, GREEN1, GREEN2)
        
        if cv2.countNonZero(red_mask1) + cv2.countNonZero(red_mask2) > 30:
            #rospy.logwarn("RED!")
            #print("RED!")
            light = TrafficLight.RED
           
            
        elif cv2.countNonZero(yellow_mask) > 30:
            #rospy.logwarn("YELLOW!")
            #print("YELLOW!")
            light = TrafficLight.YELLOW
            
        elif cv2.countNonZero(green_mask) > 30:
            #rospy.logwarn("GREEN!")
            light = TrafficLight.GREEN
            
        return light
