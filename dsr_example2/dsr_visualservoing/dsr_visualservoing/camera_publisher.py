# 
#  camera_publisher.py
#  Author: Chemin Ahn (chemx3937@gmail.com)
#  
#  Copyright (c) 2024 Doosan Robotics
#  Use of this source code is governed by the BSD, see LICENSE
# 


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        
        self.publisher_ = self.create_publisher(Image, '/real_camera/image', 10)
        self.timer = self.create_timer(0.1, self.publish_image)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)

        if not self.cap.isOpened():
            self.get_logger().error("Can't connect with camera.")
            rclpy.shutdown()

    def publish_image(self):
        ret, frame = self.cap.read()

        if ret:
            ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            self.publisher_.publish(ros_image)
            self.get_logger().info('Publishing Image')
        else:
            self.get_logger().warn("Can't read the image.")

    def destroy(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Quit the Node')
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()