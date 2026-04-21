from styx_msgs.msg import TrafficLight
import numpy as np
import rospy
import yaml
import os
from tensorflow.keras.models import load_model

IMG_HEIGHT = 600
IMG_WEIGHT = 800
IMG_CH = 3


class TLClassifier(object):
    def __init__(self):

        self.model_dir_path = None
        self.model = None

        # load configuration string
        conf_str = rospy.get_param("/traffic_light_config")

        self.configuration = yaml.safe_load(conf_str)

        # select model for Carla or simulator
        if self.configuration['is_site']:
            self.model_dir_path = "./models/site_model.h5"
        else:
            self.model_dir_path = "/home/student/CarND-Capstone/train_nn/shapes_cnn.h5"
        rospy.loginfo("model directory path: {}".format(self.model_dir_path))

        # load the model
        if not os.path.exists(self.model_dir_path):
            rospy.logerr("model directory path {} does not exist".format(self.model_dir_path))
        else:
            self.model = load_model(self.model_dir_path)
            rospy.loginfo("model loaded successfully from {}".format(self.model_dir_path))

    def get_classification(self, image):
        if self.model is None:
            rospy.logerr("Model is None")
            return TrafficLight.UNKNOWN

        image = np.reshape(image, (1, IMG_HEIGHT, IMG_WEIGHT, IMG_CH))
        score_list = self.model.predict(image)
        print("The output is : ", score_list)

        if score_list is None or len(score_list) == 0:
            return TrafficLight.UNKNOWN

        light_type = np.argmax(score_list)

        if light_type == 0:
            print("------ detecting RED-------")
            return TrafficLight.RED
        if light_type == 1:
            print("-------detecting YELLOW----")
            return TrafficLight.YELLOW
        if light_type == 2:
            print("------ detecting GREEN-----")
            return TrafficLight.GREEN
        print("------ detecting UNKNOWN-----")
        return TrafficLight.UNKNOWN
