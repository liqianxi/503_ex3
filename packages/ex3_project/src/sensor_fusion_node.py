#!/usr/bin/env python3

"""
This is the sensor fusion node for exercise 3 
"""

import numpy as np
import os
import math
import rospy
import time
import message_filters
import typing

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Pose2DStamped, LanePose, WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped
from std_msgs.msg import Header, Float32, String, Float64MultiArray,Float32MultiArray

import rosbag
import tf2_ros

# Change this before executing
VERBOSE = 0
SIM = False


class SensorFusionNode(DTROS):
    """
    Lane Control Node is used to generate control commands from lane pose estimations
    """
    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """
        # Initialize the DTROS parent class
        super(SensorFusionNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)
        if os.environ["VEHICLE_NAME"] is not None:
            self.veh_name = os.environ["VEHICLE_NAME"]
        else:
            self.veh_name = "csc22945"

        # Static parameters

        # Publishers
        ## Publish commands to the motors
        self.pub_motor_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        
        # Subscribers
        ## Subscribe to the lane_pose node
        self.sub_lane_reading = rospy.Subscriber(f"/{self.veh_name}/lane_filter_node/lane_pose", LanePose, self.cb_lane_pose, queue_size = 1)
        
        self.log("Initialized")

    # Start of callback functions
    def cb_sensor_fusion(self, input_pose_msg):
        self.pose_msg = input_pose_msg
        self.get_control_action(self.pose_msg)

if __name__ == '__main__':
    node = SensorFusionNode(node_name='sensor_fusion_node')
    # Keep it spinning to keep the node alive
    # main loop
    rospy.spin()
