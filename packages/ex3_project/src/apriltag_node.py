#!/usr/bin/env python3
import os
import rospy
import cv2
import yaml
import numpy as np
from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from sensor_msgs.msg import CompressedImage, CameraInfo
from augmented_reality_basics import Augmenter
from cv_bridge import CvBridge
from duckietown_utils import load_homography, load_map,get_duckiefleet_root
import rospkg 
from dt_apriltags import Detector, Detection
from duckietown_msgs.msg import Twist2DStamped, WheelEncoderStamped, WheelsCmdStamped,LEDPattern
from duckietown_msgs.srv import SetCustomLEDPattern, ChangePattern
# Code from https://github.com/Coral79/exA-3/blob/44adf94bad728507608086b91fbf5645fc22555f/packages/augmented_reality_basics/include/augmented_reality_basics/augmented_reality_basics.py
# https://docs.photonvision.org/en/latest/docs/getting-started/pipeline-tuning/apriltag-tuning.html


def argmin(lst):
    tmp = lst.index(min(lst))

    return tmp



class AprilTagNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(AprilTagNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh = "csc22945"

        self.rospack = rospkg.RosPack()
        #print(self.rospack.get_path('ex3_project'))
        self.read_params_from_calibration_file()
        # Get parameters from config
        self.camera_info_dict = self.load_intrinsics()

        self.augmenter = Augmenter(self.homography,self.camera_info_msg)        
        # Subscribing 
        self.sub_image = rospy.Subscriber( f'/{self.veh}/camera_node/image/compressed',CompressedImage,self.project, queue_size=10)
        
        # Publisher
        self.pub_result_ = rospy.Publisher(f'/{self.veh}/apriltag_node/modified/image/compressed', CompressedImage,queue_size=10)

        # Keep this state so you don't need to reset the same color over and over again.
        self.current_led_pattern = 4

        # extract parameters from camera_info_dict for apriltag detection
        f_x = self.camera_info_dict['camera_matrix']['data'][0]
        f_y = self.camera_info_dict['camera_matrix']['data'][4]
        c_x = self.camera_info_dict['camera_matrix']['data'][2]
        c_y = self.camera_info_dict['camera_matrix']['data'][5]
        self.camera_params = [f_x, f_y, c_x, c_y]
        K_list = self.camera_info_dict['camera_matrix']['data']
        self.K = np.array(K_list).reshape((3, 3))

        # initialise the apriltag detector
        self.at_detector = Detector(searchpath=['apriltags'],
                           families='tag36h11',
                           nthreads=1,
                           quad_decimate=2,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)

        serve_name = f"{self.veh}/led_emitter_node/set_custom_pattern"
        rospy.wait_for_service(serve_name)

        self.emitter_service = rospy.ServiceProxy(serve_name, SetCustomLEDPattern,persistent=True)

    def color_pattern(self,mode):
        msg = LEDPattern()
        if mode == 1:
            msg.color_list = ['red', 'red', 'red', 'red', 'red']
            msg.color_mask = [0, 0, 0, 0, 0]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
        elif mode==2:
            msg.color_list = ['blue', 'blue', 'blue', 'blue', 'blue']
            msg.color_mask = [0, 0, 0, 0, 0]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
        elif mode==3:
            msg.color_list = ['green', 'green', 'green', 'green', 'green']
            msg.color_mask = [0, 0, 0, 0, 0]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]
        elif mode==4:
            msg.color_list = ['white', 'white', 'white', 'white', 'white']
            msg.color_mask = [0, 0, 0, 0, 0]
            msg.frequency = 0
            msg.frequency_mask = [0, 0, 0, 0, 0]

        return msg

    def change_led(self, tag_id):
        tag_id = int(tag_id)
        if tag_id in [63, 143, 58, 62, 133, 153]:
            # T-intersection
            if self.current_led_pattern != 2:
                self.emitter_service(self.color_pattern(2))
                self.current_led_pattern = 2
        elif tag_id in [162,169]:
            # Stop sign
            if self.current_led_pattern != 1:
                self.emitter_service(self.color_pattern(1))
                self.current_led_pattern = 1
        elif tag_id in [93,94,200,201]:
            # U of A sign
            if self.current_led_pattern != 3:
                self.emitter_service(self.color_pattern(3))
                self.current_led_pattern =3

    def draw_boundary(self,corners, image):
        """
        corners = [[375.08706665 209.4493866 ]
            [445.62255859 212.48104858]
            [453.69393921 136.50134277]
            [377.13027954 138.41127014]]
        
        """
        color = (0, 0, 255)
        x_list, y_list = [],[]
        for i in range(len(corners)-1):
            start = (int(corners[i][0]),int(corners[i][1]))
            end = (int(corners[i+1][0]),int(corners[i+1][1]))
            cv2.line(image, start, end, color, 5)
        
        cv2.line(image, (int(corners[0][0]),int(corners[0][1])), 
                        (int(corners[-1][0]),int(corners[-1][1])), color, 5)
        return image

    def add_text_to_img(self, tag_id, center, img):
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        fontScale              = 0.5
        fontColor              = (255,255,255)
        thickness              = 1
        lineType               = 2

        text = "Others"
        if tag_id in [63, 143, 58, 62, 133, 153]:
            # T-intersection
            text = "T-intersection"

        elif tag_id in [162,169]:
            # Stop sign
            text = "Stop sign"

        elif tag_id in [93,94,200,201]:
            # U of A sign
            text = "U of A"


        cv2.putText(img,text, 
            (int(center[0]),int(center[1])), 
            font, 
            fontScale,
            fontColor,
            thickness,
            lineType)

        return img


    def project(self, msg):
        br = CvBridge()
        # Convert image to cv2 image.
        self.raw_image = br.compressed_imgmsg_to_cv2(msg)
        # Convert to grey image and distort it.
        dis = self.augmenter.process_image(self.raw_image)
        tags = self.at_detector.detect(dis, estimate_tag_pose=True, camera_params=self.camera_params, tag_size=0.065) # returns list of detection objects

        """
        A tag obj looks like:
        [Detection object:
            tag_family = b'tag36h11'
            tag_id = 133
            hamming = 0
            decision_margin = 48.69158172607422
            homography = [[ 2.27168755e+01  1.34799177e+01  4.11538806e+02]
            [-5.57879559e+00  4.34456107e+01  1.75621770e+02]
            [-3.39070666e-02  3.86488397e-02  1.00000000e+00]]
            center = [411.53880649 175.6217704 ]
            corners = [[375.08706665 209.4493866 ]
            [445.62255859 212.48104858]
            [453.69393921 136.50134277]
            [377.13027954 138.41127014]]
            pose_R = None
            pose_t = None
            pose_err = None
            , Detection object:
            tag_family = b'tag36h11'
            tag_id = 153
            hamming = 0
            decision_margin = 55.499420166015625
            homography = [[ 3.60020190e+01  1.26534920e+01  2.56767486e+02]
            [-3.62088538e-01  4.18653617e+01  1.80028854e+02]
            [ 5.39008255e-03  3.58126187e-02  1.00000000e+00]]
            center = [256.7674857  180.02885442]
            corners = [[226.52742004 215.69433594]
            [293.33673096 212.76560974]
            [288.90524292 142.12521362]
            [217.05523682 144.47848511]]
            pose_R = None
            pose_t = None
            pose_err = None
            ]

            Detection object:
            tag_family = b'tag36h11'
            tag_id = 162
            hamming = 0
            decision_margin = 40.54393005371094
            homography = [[-7.80885439e+00 -8.66781223e+00  5.52224980e+02]
            [-5.58210959e+00  1.12022310e+01  1.53705705e+02]
            [-4.15111722e-02 -1.00903219e-02  1.00000000e+00]]
            center = [552.22497979 153.70570474]
            corners = [[534.56939697 165.29629517]
            [564.89788818 167.99459839]
            [571.02606201 141.3631134 ]
            [540.79577637 140.81910706]]
            pose_R = [[ 0.62178591 -0.10648683  0.77591419]
            [ 0.21599374  0.97560771 -0.03919569]
            [-0.75281404  0.19196393  0.62961962]]
            pose_t = [[ 0.58169823]
            [-0.15982501]
            [ 0.75459753]]
            pose_err = 2.231643479620302e-06


            close:
            [[ 0.00283237]
            [-0.03544524]
            [ 0.1758509 ]]


            Far:
            [[ 0.01155899]
            [-0.06969869]
            [ 0.28327815]]

            Move to right:
            [[-0.0800036 ]
            [-0.05014226]
            [ 0.23373001]]

                    
                    
        """
        detection_threshold = 10 # The target margin need to be larger than this to get labelled.
        z_threshold = 0.7 # in meters
        new_img = dis
        if len(tags) == 0:
            # Means there's no tags present. Set the led to white.
            if self.current_led_pattern != 4:
                self.emitter_service(self.color_pattern(4))
                self.current_led_pattern = 4
            
        else:
            distance_list = []
            tag_list = []
            for tag in tags:
                
                
                z = tag.pose_t[2][0]
                #print(tag.tag_id, tag.decision_margin)
                if tag.decision_margin < detection_threshold or z > z_threshold:
                    continue
                #print(tag.pose_t)
                # Change led light according to the tag type.
                
                distance_list.append(tag.pose_t[2][0])
                tag_list.append(tag)
            if len(distance_list) != 0:
                #print(distance_list)
                closest_tag_idx = argmin(distance_list)
                #print("argmin",closest_tag_idx)
                tag = tag_list[closest_tag_idx]
                # Draw bounding box/
                new_img = self.draw_boundary(tag.corners, dis)

                # Add text to the center of the box.
                new_img = self.add_text_to_img(tag.tag_id, tag.center, new_img)
                self.change_led(tag.tag_id)
        
        # make new CompressedImage to publish
        augmented_image_msg = CompressedImage()
        augmented_image_msg.header.stamp = rospy.Time.now()
        augmented_image_msg.format = "jpeg"
        augmented_image_msg.data = np.array(cv2.imencode('.jpg', new_img)[1]).tostring()
        #render = self.augmenter.render_segments(points=self._points, img=dis, segments=self._segments)
        #result = br.cv2_to_compressed_imgmsg(render,dst_format='jpg')
        self.pub_result_.publish(augmented_image_msg)

    def read_params_from_calibration_file(self):
        # Get static parameters
        file_name_ex = self.get_extrinsic_filepath(self.veh)
        self.homography = self.readYamlFile(file_name_ex)
        self.camera_info_msg = rospy.wait_for_message(f'/{self.veh}/camera_node/camera_info', CameraInfo)


    def get_extrinsic_filepath(self,name):
        #TODO: retrieve the calibration info from the right path.
        cali_file_folder = self.rospack.get_path('ex3_project')+'/config/calibrations/camera_extrinsic/'

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

    def readYamlFile2(self,fname):
        """
            Reads the 'fname' yaml file and returns a dictionary with its input.

            You will find the calibration files you need in:
            `/data/config/calibrations/`
        """
        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file, Loader=yaml.Loader)
                return yaml_dict
            except yaml.YAMLError as exc:
                self.log("YAML syntax error. File: %s fname. Exc: %s"
                         %(fname, exc), type='fatal')
                rospy.signal_shutdown('No calibration file found.')
                return

    def load_intrinsics(self):
        # Find the intrinsic calibration parameters
        # cali_file_folder = '/data/config/calibrations/camera_intrinsic/'
        # self.frame_id = self.veh + '/camera_optical_frame'
        # self.cali_file = cali_file_folder + self.veh + ".yaml"

        self.cali_file = self.rospack.get_path('ex3_project') + f"/config/calibrations/camera_intrinsic/{self.veh}.yaml"

        # Locate calibration yaml file or use the default otherwise
        rospy.loginfo(f'Looking for calibration {self.cali_file}')
        if not os.path.isfile(self.cali_file):
            self.logwarn("Calibration not found: %s.\n Using default instead." % self.cali_file)
            self.cali_file = (cali_file_folder + "default.yaml")

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            rospy.signal_shutdown("Found no calibration file ... aborting")

        # Load the calibration file
        calib_data = self.readYamlFile2(self.cali_file)
        self.log("Using calibration file: %s" % self.cali_file)

        return calib_data

if __name__ == '__main__':
    augmented_reality_basics_node = AprilTagNode(node_name='apriltag_node')
    # Keep it spinning to keep the node alive
    rospy.spin()