#!/usr/bin/env python3

"""
This is the lane controller node for exercise 3 
based on the lane controller node from dt-core here: https://github.com/duckietown/dt-core/blob/daffy/packages/lane_control/src/lane_controller_node.py
"""

import numpy as np
import os
import math
import rospy
import time
import message_filters
import typing
from lane_controller import LaneController

from duckietown.dtros import DTROS, NodeType, TopicType, DTParam, ParamType
from duckietown_msgs.msg import Pose2DStamped, LanePose, WheelEncoderStamped, WheelsCmdStamped
from std_msgs.msg import Header, Float32, String, Float64MultiArray,Float32MultiArray

import rosbag


# Change this before executing
VERBOSE = 0
SIM = False


class LaneControlNode(DTROS):
    """
    Lane Control Node is used to generate control commands from lane pose estimations
    """
    def __init__(self, node_name):
        """Wheel Encoder Node
        This implements basic functionality with the wheel encoders.
        """
        # Initialize the DTROS parent class
        super(LaneControlNode, self).__init__(node_name=node_name, node_type=NodeType.DRIVER)
        if os.environ["VEHICLE_NAME"] is not None:
            self.veh_name = os.environ["VEHICLE_NAME"]
        else:
            self.veh_name = "csc22935"

        # Static parameters
        self.d_offset = 0 
        self.lane_controller_parameters = {
            "Kp_d": 0.02,
            "Ki_d": 0.01,
            "Kd_d": 0.0,
            "Kp_theta": 0.4,
            "Ki_theta": 0.075,
            "Kd_theta": 0.0,
            "sample_time": 0.01,
            "d_bounds": (-0.25,0.25),
            "theta_bounds": (-0.4,0.4),
        }

        self.forward_vel = 0.4
        


        # Initialize variables
        self.pose_msg_dict = dict()
        self.current_pose_source = "lane_filter"
        self.lane_pid_controller = LaneController(self.lane_controller_parameters)
        # Publishers
        ## Publish commands to the motors
        self.pub_motor_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        
        # Subscribers
        ## Subscribe to the lane_pose node
        self.sub_lane_reading = rospy.Subscriber(f"/{self.veh_name}/lane_filter_node/lane_pose", LanePose, self.cb_all_poses, "lane_filter", queue_size = 1)

        
        
        self.log("Initialized")

    # Start of callback functions
    def cb_all_poses(self, input_pose_msg, pose_source):
        if pose_source == self.current_pose_source:
            self.pose_msg_dict[pose_source] = input_pose_msg
            self.pose_msg = input_pose_msg
            self.get_control_action(self.pose_msg)


    def get_control_action(self, pose_msg):
        """
        Callback function that receives a pose message and updates the related control command
        """
        d_err = pose_msg.d - self.d_offset
        phi_err = pose_msg.phi

        correction = self.lane_pid_controller.compute_control_actions(d_err, phi_err, None)
 
        left_vel = self.forward_vel - correction
        right_vel = self.forward_vel + correction 
        motor_cmd = WheelsCmdStamped()
        motor_cmd.header.stamp = rospy.Time.now()
        motor_cmd.vel_left = left_vel
        motor_cmd.vel_right = right_vel
        self.pub_motor_commands.publish(motor_cmd)




    def on_shutdown(self):
        """Cleanup function."""
        while not rospy.is_shutdown():
            motor_cmd = WheelsCmdStamped()
            motor_cmd.header.stamp = rospy.Time.now()
            motor_cmd.vel_left = 0.0
            motor_cmd.vel_right = 0.0
            self.pub_motor_commands.publish(motor_cmd)

if __name__ == '__main__':
    node = LaneControlNode(node_name='lane_control_node')
    # Keep it spinning to keep the node alive
    # main loop
    rospy.spin()
