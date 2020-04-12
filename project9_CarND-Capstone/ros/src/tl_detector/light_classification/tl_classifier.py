from styx_msgs.msg import TrafficLight
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import scipy.misc
import yaml
import os
from keras.models import load_model

from keras import backend as K

IMG_HEIGHT = 600
IMG_WEIGHT = 800
IMG_CH = 3


class TLClassifier(object):
    def __init__(self):
    
        self.model_dir_path = None
        self.model = None 
        self.graph = None   
    
        # load configuration string
        conf_str = rospy.get_param("/traffic_light_config")
    
        self.configuration = yaml.load(conf_str)
        
        # select model for Carla or simulator
        if(self.configuration['is_site']):
            self.model_dir_path = "./models/site_model.h5"
        else:
            self.model_dir_path = "/home/student/CarND-Capstone/train_nn/shapes_cnn.h5"
        rospy.loginfo("model directory path: {}".format(self.model_dir_path))
        
        # load the model
        if not(os.path.exists(self.model_dir_path)):
            rospy.logerr("model directory path {} does not exist".format(self.model_dir_path))
        else:
            self.model = load_model(self.model_dir_path)
            self.model._make_predict_function()
            self.graph = K.tf.get_default_graph()
            rospy.loginfo("model loaded successfully from{}".format(self.model_dir_path))
    def get_classification(self, image):
        
        with self.graph.as_default(): 
		    light = TrafficLight.UNKNOWN
		    image = np.reshape(image, (1,IMG_HEIGHT,IMG_WEIGHT,IMG_CH))
		    
		    score_list = self.model.predict(image)
		    print "The output is : ", score_list
		    # chekc if graph is None
		    if self.graph == None:
		        rospy.logerr("Graph is None")
		        return TrafficLight.UNKNOWN
		    # check if model is empty
		    if self.model == None:
		        rospy.logerr("Model is None")
		        return TrafficLight.UNKNOWN
		    
		    # Using the trained model to check out the image
		    if( type(score_list) == None or len(score_list) == 0 ):    
		        return TrafficLight.UNKNOWN
		        
		    # non empty score_list
		    light_type = np.argmax(score_list)
		    
		    if (light_type == 0):
		        print "------ detecting RED-------"
		        return TrafficLight.RED
		    if (light_type == 1):
		        print "-------detecting YELLOW----"
		        return TrafficLight.YELLOW
		    if (light_type == 2):
		        print "------ detecting GREEN-----"
		        return TrafficLight.GREEN
		    else:
		        print "------ detecting UNKNOWN-----"
		        return TrafficLight.UNKNOWN
