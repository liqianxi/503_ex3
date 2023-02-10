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
from duckietown_utils import load_homography, load_map
import rospkg 
from dt_apriltags import Detector, Detection
# Code from https://github.com/Coral79/exA-3/blob/44adf94bad728507608086b91fbf5645fc22555f/packages/augmented_reality_basics/include/augmented_reality_basics/augmented_reality_basics.py

class AprilTagNode(DTROS):

    def __init__(self, node_name):

        # Initialize the DTROS parent class
        super(AprilTagNode, self).__init__(node_name=node_name, node_type=NodeType.PERCEPTION)
        self.veh = "csc22945"
        self.rospack = rospkg.RosPack()
        print(self.rospack.get_path('ex3_project'))
        self.read_params_from_calibration_file()
        # Get parameters from config
        self.camera_info_dict = self.load_intrinsics()
        #print(self._segments)
        self.augmenter = Augmenter(self.homography,self.camera_info_msg)        
        # Subscribing 
        self.sub_image = rospy.Subscriber( f'/{self.veh}/camera_node/image/compressed',CompressedImage,self.project, queue_size=1)
        
        # Publishersrospack
        
        self.pub_result_ = rospy.Publisher(f'/{self.veh}/apriltag_node/modified/image/compressed', CompressedImage,queue_size=1)
    

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
                           quad_decimate=1.0,
                           quad_sigma=0.0,
                           refine_edges=1,
                           decode_sharpening=0.25,
                           debug=0)
    def projection_matrix(self, K, H):
        """
            K is the intrinsic camera matrix
            H is the homography matrix
            Write here the compuatation for the projection matrix, namely the matrix
            that maps the camera reference frame to the AprilTag reference frame.
        """

        # find R_1, R_2 and t
        Kinv = np.linalg.inv(K)
        r1r2t = np.matmul(Kinv, H) # r1r2t = [r_1 r_2 t]
        r1r2t = r1r2t / np.linalg.norm(r1r2t[:, 0])
        r_1 = r1r2t[:, 0]
        r_2 = r1r2t[:, 1]
        t = r1r2t[:, 2]

        # Find v_3 vector othogonal to v_1 and v_2
        r_3 = np.cross(r_1, r_2)

        # Reconstruct R vector
        R = np.column_stack((r_1, r_2, r_3))

        # Use SVD to make R into an orthogonal matrix
        _, U, Vt = cv2.SVDecomp(R)
        R = U @ Vt

        # Combine R, t and K to find P
        buff = np.column_stack((R, t)) # buff = [r_1 r_2 r_3 t]
        P = K @ buff

        return P
    
    def project(self, msg):
        br = CvBridge()

        self.raw_image = br.compressed_imgmsg_to_cv2(msg)
        dis = self.augmenter.process_image(self.raw_image)
        tags = self.at_detector.detect(dis, estimate_tag_pose=False, camera_params=self.camera_params, tag_size=0.065) # returns list of detection objects


        # Important:
        # 133 and 153 are left and right T.
        # 94, 93, 201,200 are u of a logos :use green color
        # otherwise, use white color.
        print("tags",tags)
        """
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
                    
                    
        """
        for tag in tags:
            H = tag.homography # assume only 1 tag in image

            # Find transformation from april tag to target image frame
            P = self.projection_matrix(self.K, H)

            # Project model into image frame
            #image_np = self.renderer.render(image_np, P)
        
        # make new CompressedImage to publish
        augmented_image_msg = CompressedImage()
        augmented_image_msg.header.stamp = rospy.Time.now()
        augmented_image_msg.format = "jpeg"
        augmented_image_msg.data = np.array(cv2.imencode('.jpg', dis)[1]).tostring()
        #render = self.augmenter.render_segments(points=self._points, img=dis, segments=self._segments)
        #result = br.cv2_to_compressed_imgmsg(render,dst_format='jpg')
        self.pub_result_.publish(augmented_image_msg)

    def read_params_from_calibration_file(self):
        # Get static parameters
        file_name_ex = self.get_extrinsic_filepath(self.veh)
        self.homography = self.readYamlFile(file_name_ex)
        self.camera_info_msg = rospy.wait_for_message(f'/{self.veh}/camera_node/camera_info', CameraInfo)


    def get_extrinsic_filepath(self,name):

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