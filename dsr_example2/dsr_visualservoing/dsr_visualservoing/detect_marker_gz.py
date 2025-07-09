# 
#  detect_marker_gz.py
#  Author: Chemin Ahn (chemx3937@gmail.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 

import rclpy
from rclpy.node import Node
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32, Float32MultiArray
from tf2_ros import TransformBroadcaster, TransformStamped
import tf_transformations
import yaml
import time
from ament_index_python.packages import get_package_share_directory
import os
from dsr_msgs2.msg import ServojStream





class DetectMarkerGz(Node):
    def __init__(self):
        super().__init__('detect_marker_gz')
        self.br = TransformBroadcaster(self)
        self.bridge = CvBridge()
        self.aruco_dict = cv.aruco.Dictionary_get(cv.aruco.DICT_4X4_1000)
        self.aruo_params = cv.aruco.DetectorParameters_create()
        calibration_file = self.get_parameter_or("calibration_file",os.path.join(get_package_share_directory('dsr_visualservoing'), 'config', 'rgbd_camera_gz.yaml'))

        with open(calibration_file, "r") as file:
            calib_data = yaml.safe_load(file)
            self.camera_matrix = np.array(calib_data["camera_matrix"]["data"]).reshape((3, 3))
            self.dist_coeffs = np.array(calib_data["distortion_coefficients"]["data"])

        self.current_marker_id = None
        self.previous_marker_id = None
        self.marker_detected_time = None
        self.marker_size = 0.115

        self.marker_detected_time = None

        self.current_marker_id = None
        self.previous_marker_id = None

        self.current_xyz_msg = None
        self.previous_xyz_msg = None

        self.current_rxyz_msg = None
        self.previous_rxyz_msg = None

        self.current_marker_pose = None
        self.previous_marker_pose = None

        self.last_detected_time = time.time()


        self.joint_pose = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
        self.joint_vel = [60.0, 60.0, 60.0, 60.0, 60.0, 60.0]
        self.joint_acc = [30.0, 30.0, 30.0, 30.0, 30.0, 30.0]
        self.joint_time = 0.0


        self.image_sub = self.create_subscription(Image, "/rgbd_camera/image", self.callback, 10)
        self.joint_sub = self.create_subscription(JointState, "/dsr01/gz/joint_states", self.joint_callback, 10)

        self.marker_id_pub = self.create_publisher(Int32, '/marker/id', 10)
        self.marker_pose_pub = self.create_publisher(Float32MultiArray, '/marker/pose', 10)


    def joint_callback(self, msg):
        self.current_joint = list(msg.position)
        
        # joint 순서 조정: 1,2,4,5,3,6 -> 1,2,3,4,5,6
        self.current_joint[2],self.current_joint[3], self.current_joint[4] = self.current_joint[4],self.current_joint[2],  self.current_joint[3]  
        self.current_joint = [round(j * 180 / np.pi, 1) for j in self.current_joint] 


    def callback(self, data):     
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")  
        except CvBridgeError as e:
            self.get_logger().error("CvBridgeError: {0}".format(e))
            return
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        ret = cv.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruo_params)
        corners, ids = ret[0], ret[1]

       
        if len(corners) > 0 and ids is not None:
            if self.current_marker_id is None or self.current_marker_id not in ids:
                self.current_marker_id = ids[0][0]
                self.last_detected_time = time.time()

            marker_index = np.where(ids == self.current_marker_id)[0][0]

            ret = cv.aruco.estimatePoseSingleMarkers(
                [corners[marker_index]], self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            (rvec, tvec) = (ret[0], ret[1])

            tvec4drawAxis = np.round(tvec*1000, 4)
            rvec4drawAxis= np.round(rvec, 4)

            tvec[0, 0, 0] = tvec[0, 0, 0]
            tvec[0, 0, 1] = tvec[0, 0, 1]
            tvec[0, 0, 2] = tvec[0, 0, 2]


            R_z_180 = tf_transformations.euler_matrix(0, 0, np.pi)[:3, :3]
            R_y_180 = tf_transformations.euler_matrix(0, np.pi, 0)[:3, :3]
            R_z_90 = tf_transformations.euler_matrix(0, 0, np.pi/2)[:3, :3]
            rotation = np.dot(R_z_180, R_y_180)
            rotation = np.dot(rotation, R_z_90)


            pose_cobot_camera = np.array([1.0, 0.0, 1.2])


            T_cobot_camera = np.eye(4)
            T_cobot_camera[:3, :3] = rotation
            T_cobot_camera[:3, 3] = pose_cobot_camera
            T_cobot_camera= np.round(T_cobot_camera, 9)


            tvec_homogeneous = np.append(tvec[0, 0, :], 1.0).reshape(4,1)
            tvec_cobot_frame = np.dot(T_cobot_camera, tvec_homogeneous) 
            tvec_cobot_frame = tvec_cobot_frame[:3]
            tvec_cobot_frame = tvec_cobot_frame * 1000
            tvec_cobot_frame= np.round(tvec_cobot_frame,  4)  


            R_marker_camera, _ = cv.Rodrigues(rvec[0])
            R_marker_cobot = np.dot(rotation, R_marker_camera)
            rpy_cobot = tf_transformations.euler_from_matrix(R_marker_cobot)
            rpy_cobot= np.array(rpy_cobot)
            rpy_cobot= np.round(rpy_cobot * (180/np.pi), 4)


            self.current_xyz_msg = Vector3(x=float(tvec_cobot_frame[0]), y=float(tvec_cobot_frame[1]), z=float(tvec_cobot_frame[2]+ 200))
            self.current_rxyz_msg = Vector3(x=rpy_cobot[0], y=rpy_cobot[1]+180, z=rpy_cobot[2])
            self.current_marker_pose = Float32MultiArray()
            self.current_marker_pose.data = [self.current_xyz_msg.x, self.current_xyz_msg.y, self.current_xyz_msg.z,
                                             self.current_rxyz_msg.x, self.current_rxyz_msg.y, self.current_rxyz_msg.z]

            current_time = self.get_clock().now()

            self.marker_pose_pub.publish(self.current_marker_pose)
            self.marker_id_pub.publish(Int32(data=int(self.current_marker_id)))
            self.previous_marker_id = self.current_marker_id

            self.last_published_id = self.current_marker_id
            self.last_published_xyz = self.current_xyz_msg
            self.last_published_rxyz = self.current_rxyz_msg

            cv.aruco.drawDetectedMarkers(cv_image, [corners[marker_index]])  
            cv.aruco.drawAxis(
                cv_image,
                self.camera_matrix,
                self.dist_coeffs,
                rvec4drawAxis[0, :, :],
                tvec4drawAxis[0, :, :],
                50
            )

        else:
            current_time = time.time()
            time_since_last_detected = current_time - self.last_detected_time
            if time_since_last_detected > 2:
                self.current_marker_id = 1000

                self.joint_pose_pub = self.create_publisher(ServojStream, '/dsr01/servoj_stream', 10)

                if self.current_joint != self.joint_pose:
                    self.joint_msg = ServojStream()
                    self.joint_msg.pos = self.joint_pose
                    self.joint_msg.vel = self.joint_vel
                    self.joint_msg.acc = self.joint_acc
                    self.joint_msg.time = 0.0
                    self.joint_pose_pub.publish(self.joint_msg)


                else:
                    self.destroy_publisher(self.joint_pose_pub)
 

                self.get_logger().info("Failed Detecting Marker. Move to Waiting Pose and Shut down the Node.")
                self.marker_id_pub.publish(Int32(data=int(self.current_marker_id)))
                self.previous_marker_id = self.current_marker_id

        cv.imshow("Image", cv_image)
        if cv.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("User requested shutdown, shutting down node.")
            self.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectMarkerGz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down chem_detect_marker_gz_continue node.")
        cv.destroyAllWindows()
        node.destroy_node()
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
