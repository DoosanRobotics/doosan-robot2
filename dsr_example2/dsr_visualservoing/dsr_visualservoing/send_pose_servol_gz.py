# 
#  send_pose_servol_gz.py
#  Author: Chemin Ahn (chemx3937@gmail.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

import rclpy
from rclpy.node import Node
from dsr_msgs2.msg import ServolStream
from std_msgs.msg import Int32, Float32MultiArray
import numpy as np
import math

class SendPoseServoLGz(Node):

    def __init__(self):
        super().__init__('send_pose_servol_gz') 

        self.marker_id_sub = self.create_subscription(Int32, '/marker/id', self.marker_id_callback, 10)
        self.marker_pose_sub = self.create_subscription(Float32MultiArray, '/marker/pose', self.marker_pose_callback, 10)

        self.current_marker_id = None
        self.current_desired_pose = None

        self.previous_marker_id = None
        self.previous_desired_pose = None


    def marker_id_callback(self, msg):
        self.current_marker_id = msg.data
        self.check_and_process_data()

    def marker_pose_callback(self, msg):
        self.current_desired_pose= msg.data
        self.check_and_process_data()


    def check_and_process_data(self):
        if self.current_marker_id is not None and self.current_desired_pose is not None:
            self.detected_marker_servol_pub = self.create_publisher(ServolStream, '/dsr01/servol_stream', 10)
 

            if self.current_marker_id == 1000:
                self.destroy_publisher(self.detected_marker_servol_pub)
                self.get_logger().info("Failed Detecting Marker. Move to waiting Position.")
            
            else:
                self.publish_pose()

                if self.previous_marker_id == None:
                    self.get_logger().info("First Detect, Move to target pose.")
                    self.get_logger().info(f"ID: {self.current_marker_id}, Pose: {self.current_desired_pose}")
                    pass


                else:
                    if self.previous_marker_id == self.current_marker_id:
                        if self.previous_desired_pose[0:3] != self.current_desired_pose[0:3]:
                            self.get_logger().info("Same Marker, Different Pose, Send New Pose")
                            self.get_logger().info(f"ID: {self.current_marker_id}, Pose: {self.current_desired_pose}")

                    else:
                        self.get_logger().info("Differnt Marker")
                        self.get_logger().info(f"ID: {self.current_marker_id}, Pose: {self.current_desired_pose}")

            self.previous_marker_id = self.current_marker_id
            self.previous_desired_pose = self.current_desired_pose                


    def publish_pose(self):

        self.msg = ServolStream()
        self.msg.pos = self.current_desired_pose
        self.msg.vel = [500.0, 180.0]
        self.msg.acc = [875.0, 180.0]
        self.msg.time = 0.0
        
        self.detected_marker_servol_pub.publish(self.msg)


def main(args=None):
    rclpy.init(args=args)
    node = SendPoseServoLGz()

    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
