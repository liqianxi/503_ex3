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
from duckietown_msgs.msg import Pose2DStamped, LanePose, WheelEncoderStamped, WheelsCmdStamped, Twist2DStamped
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
            self.veh_name = "csc22945"

        # Static parameters
        self.d_offset = 0.0
        self.lane_controller_parameters = {
            "Kp_d": 10.0,
            "Ki_d": 0.75,
            "Kd_d": 0.25,
            "Kp_theta": 5.0,
            "Ki_theta": 0.25,
            "Kd_theta": 0.125,
            "sample_time": 0.01,
            "d_bounds": (-2.0, 2.0),
            "theta_bounds": (-2.0,2.0),
        }


        # Initialize variables
        self.pose_msg_dict = dict()
        self.lane_pid_controller = LaneController(self.lane_controller_parameters)

        # Publishers
        ## Publish commands to the motors
        self.pub_motor_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        self.pub_car_cmd = rospy.Publisher(f'/{self.veh_name}/car_cmd_switch_node/cmd', Twist2DStamped, queue_size=1, dt_topic_type=TopicType.CONTROL)
        
        # Subscribers
        ## Subscribe to the lane_pose node
        self.sub_lane_reading = rospy.Subscriber(f"/{self.veh_name}/lane_filter_node/lane_pose", LanePose, self.cb_lane_pose, queue_size = 1)

        
        
        self.log("Initialized")

    # Start of callback functions
    def cb_lane_pose(self, input_pose_msg):
        self.pose_msg = input_pose_msg
        self.get_control_action(self.pose_msg)


    def get_control_action(self, pose_msg):
        """
        Callback function that receives a pose message and updates the related control command
        """
        d_err = pose_msg.d - self.d_offset
        phi_err = pose_msg.phi

        v, omega = self.lane_pid_controller.compute_control_actions(d_err, phi_err, None)

        # Initialize car control message
        car_control_msg = Twist2DStamped()
        car_control_msg.header = pose_msg.header

        car_control_msg.v = v
        car_control_msg.omega = omega * 2.5
        print("omega:",omega)
        self.pub_car_cmd.publish(car_control_msg)






    def on_shutdown(self):
        """Cleanup function."""
        while not rospy.is_shutdown():
            motor_cmd = WheelsCmdStamped()
            motor_cmd.header.stamp = rospy.Time.now()
            motor_cmd.vel_left = 0.0
            motor_cmd.vel_right = 0.0
            self.pub_motor_commands.publish(motor_cmd)
            car_control_msg = Twist2DStamped()
            car_control_msg.header.stamp = rospy.Time.now()
            car_control_msg.v - 0.0
            car_control_msg.omega = 0.0
            self.pub_car_cmd.publish(car_control_msg)

if __name__ == '__main__':
    node = LaneControlNode(node_name='lane_control_node')
    # Keep it spinning to keep the node alive
    # main loop
    rospy.spin()
