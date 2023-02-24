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
from PID import PID


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

        # Get static parameters
        self.d_offset = 0.0
        self.L = 0.05
        self.update_rate = 30 # 30 Hz

        # Velocity settings
        self.forward_vel = 0.6
        self.rotation_vel = 0.6
        
        # Publishers
        ## Publish commands to the motors
        self.pub_motor_commands = rospy.Publisher(f'/{self.veh_name}/wheels_driver_node/wheels_cmd', WheelsCmdStamped, queue_size=1)
        
        # Subscribers
        ## Subscribe to the lane_pose node
        self.sub_lane_reading = rospy.Subscriber(f"/{self.veh_name}/lane_filter_node/lane_pose", LanePose, self.cb_all_poses, "lane_filter", queue_size = 1)
        self.sub_wheels_cmd_executed = rospy.Subscriber(f"/{self.veh_name}/wheels_driver_node/wheel_cmd", self.cb_wheels_cmd_executed, queue_size = 1)

        ## From here: https://github.com/duckietown/dt-core/blob/daffy/packages/deadreckoning/src/deadreckoning_node.py
        ## Subscribers to the wheel encoders
        self.sub_encoder_ticks_left = message_filters.Subscriber(f'/{self.veh_name}/left_wheel_encoder_node/tick', WheelEncoderStamped)
        self.sub_encoder_ticks_right = message_filters.Subscriber(f'/{self.veh_name}/right_wheel_encoder_node/tick', WheelEncoderStamped)
        ## Setup the time syncs 
        self.ts_encoders = message_filters.ApproximateTimeSynchronizer(
            [self.sub_encoder_ticks_left, self.sub_encoder_ticks_right], 10, 0.5
        )

        self.ts_encoders.registerCallback(self.cb_encoder_data)

        # For PID controller
        self.Kp = 0.4
        self.Ki = 0.075
        self.Kd = 0.00
        self.sample_time = 1.0/self.update_rate
        self.output_limits = (-0.3, 0.3)
        self.pid_controller = PID(
            Kp=self.Kp,
            Ki=self.Ki,
            Kd=self.Kd, 
            sample_time=self.sample_time, 
            output_limits=self.output_limits)

        # Store the target state for each command received
        # Store the initial state for startup
        self.x_target_hist = [0]
        self.y_target_hist = [0]
        self.yaw_target_hist = [math.pi/2.0]
        self.time_target_hist = [rospy.get_time()]

        ##
        self.ticks_per_meter = 656
        self.encoder_stale_dt = 1.0 

        self.left_encoder_last = None
        self.right_encoder_last = None
        self.encoders_timestamp_last = None
        self.encoders_timestamp_last_local = None        

        self.timestamp = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = math.pi/2.0
        self.tv = 0.0
        self.rv = 0.0
        self.lwv = 0.0
        self.rwv = 0.0
        
        # Setup timer
        self.publish_hz = self.update_rate
        self.timer = rospy.Timer(rospy.Duration(1 / self.publish_hz), self.cb_timer)
        self._print_time = 0
        self._print_every_sec = 30
        ##
        
        
        
        self.log("Initialized")

    # Start of callback functions
    def cb_all_poses(self, input_pose_msg, pose_source):

        if pose_source == self.current_pose_source:
            self.pose_msg_dict[pose_source] = input_pose_msg
            self.pose_msg = input_pose_msg
            self.get_control_action(self.pose_msg)



    def cb_encoder_data(self, left_encoder, right_encoder):
        """
        Update encoder distance information from ticks.
        From here: https://github.com/duckietown/dt-core/blob/daffy/packages/deadreckoning/src/deadreckoning_node.py#L101

        left_encoder:  Left wheel encoder ticks
        right_encoder: Right wheel encoder ticks
        """
        timestamp_now = rospy.get_time()

        # Use the average of the two encoder times as the timestamp
        left_encoder_timestamp = left_encoder.header.stamp.to_sec()
        right_encoder_timestamp = right_encoder.header.stamp.to_sec()
        timestamp = (left_encoder_timestamp + right_encoder_timestamp) / 2

        if not self.left_encoder_last:
            self.left_encoder_last = left_encoder
            self.right_encoder_last = right_encoder
            self.encoders_timestamp_last = timestamp
            self.encoders_timestamp_last_local = timestamp_now
            return

        # Skip this message if the time synchronizer gave us an older message
        dtl = left_encoder.header.stamp - self.left_encoder_last.header.stamp
        dtr = right_encoder.header.stamp - self.right_encoder_last.header.stamp
        if dtl.to_sec() < 0 or dtr.to_sec() < 0:
            # self.loginfo("Ignoring stale encoder message")
            return

        left_dticks = left_encoder.data - self.left_encoder_last.data
        right_dticks = right_encoder.data - self.right_encoder_last.data

        left_distance = left_dticks * 1.0 / self.ticks_per_meter
        right_distance = right_dticks * 1.0 / self.ticks_per_meter

        # Displacement in body-relative x-direction
        dist = (left_distance + right_distance) / 2

        # Change in heading
        dyaw = (right_distance - left_distance) / (2 * self.L)

        dt = timestamp - self.encoders_timestamp_last

        if dt < 1e-6:
            self.logwarn("Time since last encoder message (%f) is too small. Ignoring" % dt)
            return

        self.tv = dist / dt
        self.rv = dyaw / dt
        self.lwv = left_distance / dt
        self.rwv = right_distance / dt


        self.yaw = self.angle_clamp(self.yaw + dyaw)
        self.x = self.x + dist * math.cos(self.yaw)
        self.y = self.y + dist * math.sin(self.yaw)
        self.timestamp = timestamp

        f = Pose2DStamped()

        f.x = self.x
        f.y = self.y
        f.theta = self.yaw

        f.header.stamp = rospy.Time.now()
        self.pub_world_frame.publish(f)

        self.left_encoder_last = left_encoder
        self.right_encoder_last = right_encoder
        self.encoders_timestamp_last = timestamp
        self.encoders_timestamp_last_local = timestamp_now

    
    def cb_timer(self, _):
        """
        Callback for the timer

        From here: https://github.com/duckietown/dt-core/blob/daffy/packages/deadreckoning/src/deadreckoning_node.py#L173
        """
        need_print = time.time() - self._print_time > self._print_every_sec
        if self.encoders_timestamp_last:
            dt = rospy.get_time() - self.encoders_timestamp_last_local
            if abs(dt) > self.encoder_stale_dt:
                if need_print:
                    self.logwarn(
                        "No encoder messages received for %.2f seconds. "
                        "Setting translational and rotational velocities to zero" % dt
                    )
                self.rv = 0.0
                self.tv = 0.0
        else:
            if need_print:
                self.logwarn(
                    "No encoder messages received. " "Setting translational and rotational velocities to zero"
                )
            self.rv = 0.0
            self.tv = 0.0
        
    def cb_state_control_comm(self, msg):
        """
        Decode command from state control and call the right function with the given parameters
        """
        command = msg.data
        print("State Control Comm:", command)
        command_args =  command.split(":")
        if command_args[0].lower() == "forward":
            dist = float(command_args[1])
            self.get_new_target_world_frame(rotate=None, dist=dist)
            self._move_forward(dist)
        elif command_args[0].lower() == "right":
            rotation = float(command_args[1])
            self.get_new_target_world_frame(rotate=-rotation, dist=None)
            self._rotate_robot(-rotation)
        elif command_args[0].lower() == "left":
            rotation = float(command_args[1])
            self.get_new_target_world_frame(rotate=rotation, dist=None)
            self._rotate_robot(rotation)
        # fix this later
        elif command_args[0].lower() == "arc_left":
            rotation = float(command_args[1])
            radius = float(command_args[2])
            self._arc_robot(rotation, radius)
        elif command_args[0].lower() == "arc_right":
            rotation = float(command_args[1])
            radius = float(command_args[2])
            self._arc_robot(-rotation, radius)
        elif command_args[0].lower() == "shutdown":
            print("Motor Control Node shutting down")
            rospy.signal_shutdown("Motor Control Node Shutdown command received")
        else:
            print("Not a valid command")
    # End of Callback functions

    def get_control_action(self, pose_msg):
        """
        Function that receives a pose message and updates the related control command
        """
        

    def on_shutdown(self):
        """Cleanup function."""
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
