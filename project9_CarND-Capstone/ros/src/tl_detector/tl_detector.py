#!/usr/bin/env python
from timeit import default_timer as timer
from scipy.spatial import KDTree
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# When NN is on, take the comment out below
#from light_classification.tl_classifier import TLClassifier

import rospy
import tf
import cv2
import yaml
import math
import numpy as np

STATE_COUNT_THRESHOLD = 3
CAMERA_PROCESS_RATE = 0.5
WAYPOINT_AFAR = 200

IMG_HEIGHT = 600
IMG_WIDTH = 800

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
        
        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []
        
        self.trafiic_light_waypoint_indexes = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        # deleting sub6 doesnt solve the problem
        
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)
        
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        
        self.bridge = CvBridge()
        # When NN is take the comment out below
        #self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_light_state = None
        self.last_wp = -1
        self.state_count = 0
        self.has_image = False
        self.last_img_processed = 0

        # for collecting data
        #self.CLDT = 520
        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        rospy.loginfo("waypoint_cb triggered:")
        self.waypoints = waypoints
        if not self.waypoints_2d: # waypoints not initiallized yet
        	self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
        	self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights
        
        if(self.waypoint_tree == None):
            return
            
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        
        self.state_count += 1
    
    def write_image(self,state,image):
        
        # collecting data            
        self.CLDT += 1
        image_path = '/home/student/CarND-Capstone/train_nn/data/database/image_{}.png'.format(self.CLDT)
        
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        
        cv2.imwrite(image_path,image)
        
        # debug and health check
        print "Image stored successfully at : ", image_path
    
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        
        if(self.waypoint_tree == None):
            return;
        
        time_interval = timer() - self.last_img_processed
        
        # process images with intervals
        #if(time_interval < CAMERA_PROCESS_RATE):
        #    return;
        #rospy.loginfo("image_cb : time_interval {}".format(time_interval))
        
        self.has_image = True
        self.camera_image = msg
        
        self.last_img_processed = timer()
        
        light_wp, state = self.process_traffic_lights()
       
        # Capture images to /database folder when activated
        #self.write_image(state,self.camera_image)

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        
        self.state_count += 1
        

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        """
        
        closest_idx = -1
        closest_dist = -1
        a = (x,y)
        distance = lambda a,b: (a[0] - b[0])**2 + (a[1] - b[1]) ** 2
        if self.waypoints is not None:
        	p = self.waypoints.waypoints
        	for i in range(len(p)):
        		b = (p[i].pose.pose.position.x, p[i].pose.pose.position.y)
        		d = distance(a,b)
        		
        		if(closest_dist == -1) or (closest_dist > d):
        			closest_dist = d
        			closest_index = i
        			
        return closest_idx        
        """
        
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]
        return closest_idx
        

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
        # When camera is off, and car uses signals from TrafficLight, turn this on.
        return light.state
        
        # When camera is on, turn this on
        
        """
        if not (self.has_image):
            self.prev_light_loc = None
            #return False
            return light.UNKNOWN

        # generate image
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        cv_image = cv2.resize(cv_image,(IMG_WIDTH,IMG_HEIGHT))
        
        # get the light state
        result = self.light_classifier.get_classification(cv_image)  
        
        if(self.last_light_state != result):
            rospy.loginfo("Image classification {}".format(result))
            self.last_light_state = result
        return result
        """
        
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        
        if self.pose and self.waypoint_tree:
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
        		line = stop_line_positions[i]
        		
        		# get the stop light line way point index
        		temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
        		d = temp_wp_idx - car_wp_idx
        		if d >= 0 and d < diff:
        			diff = d
        			closest_light = light
        			state = closest_light.state
        			line_wp_idx = temp_wp_idx
        			
        if (closest_light) and ((line_wp_idx - car_wp_idx) <= WAYPOINT_AFAR):
            # when camera is on and NN is oon, take out the comment out below
            #state = self.get_light_state(closest_light)
            return (line_wp_idx, state)
        else:
            #self.waypoints = None # check on this
            return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
