#!/usr/bin/env python3
import os
import rospy
import cv2
import yaml
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage, CameraInfo
from augmented_reality_basics import Augmenter
from cv_bridge import CvBridge
from duckietown_utils import load_homography, load_map
import rospkg 
# Code from https://github.com/Coral79/exA-3/blob/44adf94bad728507608086b91fbf5645fc22555f/packages/augmented_reality_basics/include/augmented_reality_basics/augmented_reality_basics.py

class AugmentedRealityBasicsNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(AugmentedRealityBasicsNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh = "csc22945"

        self.read_params_from_calibration_file()
        # Get parameters from config
        self._points = rospy.get_param(
            '~points',
            dt_help="Framerate at which images frames are produced"
        )
        print(self._points)

        self._segments = rospy.get_param(
            '~segments',
            dt_help="Framerate at which images frames are produced"
        )
        print(self._segments)
        self.augmenter = Augmenter(self.homography,self.camera_info_msg)        
        # Subscribing 
        self.sub_image = rospy.Subscriber( f'/{self.veh}/camera_node/image/compressed',CompressedImage,self.project, queue_size=1)
        
        # Publishers
        self.pub_result_ = rospy.Publisher(f'/{self.veh}/augmented_reality_basics_node/calibration_pattern/image/compressed', CompressedImage,queue_size=1)
    def project(self,msg):
        br = CvBridge()
        self.raw_image = br.compressed_imgmsg_to_cv2(msg)
        dis = self.augmenter.process_image(self.raw_image)
        render = self.augmenter.render_segments(points=self._points, img=dis, segments=self._segments)
        result = br.cv2_to_compressed_imgmsg(render,dst_format='jpg')
        self.pub_result_.publish(result)

    def read_params_from_calibration_file(self):
        # Get static parameters
        file_name_ex = self.get_extrinsic_filepath(self.veh)
        self.homography = self.readYamlFile(file_name_ex)
        self.camera_info_msg = rospy.wait_for_message(f'/{self.veh}/camera_node/camera_info', CameraInfo)


    def get_extrinsic_filepath(self,name):
        rospack = rospkg.RosPack()
        cali_file_folder = rospack.get_path('augmented_reality_basics')+'/config/calibrations/camera_extrinsic/'
        print(cali_file_folder)
        cali_file = cali_file_folder + name + ".yaml"
        return cali_file

    def readYamlFile(self,fname):
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)["homography"]
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown()
                return


if __name__ == '__main__':
    augmented_reality_basics_node = AugmentedRealityBasicsNode(node_name='augmented_reality_basics_node')
    # Keep it spinning to keep the node alive
    rospy.spin()